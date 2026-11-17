// Fixed-size network-order protocol. No SDK, ROS, model or CAN dependency.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace wheel {
using Axes = std::array<double, 16>;
constexpr size_t stream_packet_size = 336;
constexpr int64_t stream_feedback_age_ns = 150000000;
// 图像有效期独立于 150ms 指令/反馈看门狗；不得用新指令序号刷新旧图时间。
constexpr int64_t stream_image_age_ns = 2000000000;

inline uint64_t network_u64(const unsigned char* bytes) {
  uint64_t value = 0;
  for (int index = 0; index < 8; ++index) value = (value << 8) | bytes[index];
  return value;
}

inline double network_double(const unsigned char* bytes) {
  const auto bits = network_u64(bytes);
  double value;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

class JointStreamGuard {
  std::string key_;
  Axes lower_{}, upper_{}, speed_{}, hold_{};
  int64_t started_ = 0, received_ = 0, image_ = 0, feedback_ = 0;
  std::array<unsigned char, 16> pair_{};
  Axes velocity_{};
  double acceleration_ = .5;
  bool fixed_image_ = false;
  bool planned_trajectory_ = false;
 public:
  uint64_t sequence = 0;
  Axes goal{};
  void begin(const std::string& key, const Axes& lo, const Axes& hi,
             const Axes& speeds, const Axes& nominal, int64_t now, double acceleration = .5) {
    if (key.size() != 32 || key.find_first_not_of("0123456789abcdef") != std::string::npos)
      throw std::invalid_argument("A new 128-bit stream key is required");
    if (!std::isfinite(acceleration) || acceleration <= 0 || acceleration > 2)
      throw std::invalid_argument("Invalid final reference acceleration bound");
    key_ = key;
    lower_ = lo; upper_ = hi; speed_ = speeds; hold_ = nominal; goal = nominal;
    started_ = now; received_ = image_ = feedback_ = 0; sequence = 0; pair_.fill(0);
    velocity_.fill(0); acceleration_ = acceleration; fixed_image_ = false; planned_trajectory_ = false;
  }

  bool accept(const unsigned char* bytes, size_t size, int64_t now, const Axes& output) {
    const bool planned = size == stream_packet_size && std::memcmp(bytes, "WJPLAN1\0", 8) == 0;
    const bool fixed = size == stream_packet_size && std::memcmp(bytes, "WJFIXD1\0", 8) == 0;
    if (expired(now) || size != stream_packet_size ||
        (!fixed && !planned && std::memcmp(bytes, "WJSTRM1\0", 8) != 0) ||
        key_.size() != 32 || std::memcmp(bytes + 8, key_.data(), 32) != 0) return false;
    const auto seq = network_u64(bytes + 40);
    const auto feedback = network_u64(bytes + 48);
    const auto image = network_u64(bytes + 56);
    if (!seq || seq <= sequence || feedback > static_cast<uint64_t>(now) ||
        image > static_cast<uint64_t>(now) || feedback < static_cast<uint64_t>(feedback_) ||
        image < static_cast<uint64_t>(image_) ||
        static_cast<uint64_t>(now) - feedback > stream_feedback_age_ns ||
        ((!sequence || (!fixed_image_ && !planned_trajectory_)) && static_cast<uint64_t>(now) - image > stream_image_age_ns)) return false;
    const bool same_pair = std::memcmp(bytes + 64, pair_.data(), 16) == 0;
    if (sequence && ((image == static_cast<uint64_t>(image_)) != same_pair)) return false;
    // 固定目标首包仍检查2s；此后只允许同一模式、原始图像时间和帧标识。
    if (sequence && (fixed != fixed_image_ || planned != planned_trajectory_ || ((fixed || planned) &&
        (image != static_cast<uint64_t>(image_) || !same_pair)))) return false;
    Axes candidate{};
    for (int axis = 0; axis < 16; ++axis) {
      candidate[axis] = network_double(bytes + 80 + axis * 8);
      const double velocity = network_double(bytes + 208 + axis * 8);
      if (!std::isfinite(candidate[axis]) || !std::isfinite(velocity) ||
          candidate[axis] < lower_[axis] || candidate[axis] > upper_[axis] ||
          std::abs(velocity) > speed_[axis] + 1e-8) return false;
      if (axis % 8 == 7) {
        if (std::abs(candidate[axis] - hold_[axis]) > 1e-8 || std::abs(velocity) > 1e-8) return false;
      } else if (std::abs(candidate[axis] - (sequence ? goal[axis] : output[axis])) > speed_[axis] * .15 + .002) {
        // Compare consecutive references, not the lagging board interpolator.
        // sample() still independently bounds the actual output speed/acceleration.
        return false;  // An oversized reference jump cannot become a new trajectory.
      }
    }
    // Planned sessions carry admission time/session ID, not an image timestamp.
    // Image admission is checked before planning by the gateway. Neither source
    // time nor mode may be renewed inside an accepted trajectory session.
    if (planned && !sequence && image < static_cast<uint64_t>(started_)) return false;
    planned_trajectory_ = planned;
    fixed_image_ = fixed;
    sequence = seq; feedback_ = feedback; image_ = image; received_ = now;
    std::memcpy(pair_.data(), bytes + 64, 16);
    goal = candidate;
    return true;
  }

  const char* expired(int64_t now) const {
    if (!sequence) return now - started_ > 1000000000 ? "joint_stream_start_timeout" : nullptr;
    if (now < received_ || now - received_ > stream_feedback_age_ns)
      return "joint_stream_command_timeout";
    if (now < feedback_ || now - feedback_ > stream_feedback_age_ns)
      return "joint_stream_feedback_timeout";
    if (now < image_ || (!fixed_image_ && !planned_trajectory_ && now - image_ > stream_image_age_ns))
      return "joint_stream_image_timeout";
    return nullptr;
  }

  // Last-resort reference guards for packet jitter; no IK or path planning.
  Axes sample(const Axes& output, double dt) {
    if (!std::isfinite(dt) || dt <= 0 || dt > .03)
      throw std::invalid_argument("Board joint stream deadline missed");
    Axes next = output;
    Axes next_velocity = velocity_;
    for (int axis = 0; axis < 16; ++axis) {
      if (axis % 8 == 7) continue;
      const double requested = std::clamp((goal[axis] - output[axis]) / dt, -speed_[axis], speed_[axis]);
      next_velocity[axis] += std::clamp(requested - velocity_[axis], -acceleration_ * dt, acceleration_ * dt);
      next[axis] += next_velocity[axis] * dt;
      if (next[axis] < lower_[axis] || next[axis] > upper_[axis])
        throw std::invalid_argument("Final stream reference outside original limits");
    }
    velocity_ = next_velocity;
    return next;
  }
};

class JointStreamSocket {
  int fd_ = -1;
 public:
  JointStreamSocket() = default;
  JointStreamSocket(const JointStreamSocket&) = delete;
  JointStreamSocket& operator=(const JointStreamSocket&) = delete;
  ~JointStreamSocket() { if (fd_ >= 0) close(fd_); }
  void open(const std::string& host, int port) {
    if (fd_ >= 0 || port < 1024 || port > 65535) throw std::invalid_argument("Invalid stream listener");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1)
      throw std::invalid_argument("Numeric IPv4 bind address required");
    const int socket_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_fd < 0) throw std::runtime_error("Cannot open joint stream socket");
    int buffer_size = 8192;
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
    if (bind(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      close(socket_fd);
      throw std::runtime_error("Cannot bind joint stream socket");
    }
    fd_ = socket_fd;
  }
  void drain(JointStreamGuard& guard, int64_t now, const Axes& output) {
    // Bound per-tick receive work and never allocate/queue trajectories.
    for (int count = 0; count < 64; ++count) {
      std::array<unsigned char, stream_packet_size + 1> bytes{};
      const auto size = recv(fd_, bytes.data(), bytes.size(), MSG_TRUNC);
      if (size < 0) break;
      guard.accept(bytes.data(), static_cast<size_t>(size), now, output);
    }
  }
};
}  // namespace wheel
