// Native receiver regression driver. No ROS, CAN or motor access.
#include "joint_stream.hpp"
#include <iostream>
#include <iomanip>

int main() {
  std::cout << std::setprecision(17);
  wheel::Axes lo{}, hi{}, speed{}, output{};
  lo.fill(-2.); hi.fill(2.); speed.fill(.2);
  wheel::JointStreamGuard guard;
  guard.begin(std::string(32, 'a'), lo, hi, speed, output, 1000000000);
  std::array<unsigned char, 8 + wheel::stream_packet_size> record{};
  while (std::cin.read(reinterpret_cast<char*>(record.data()), record.size())) {
    const auto now = static_cast<int64_t>(wheel::network_u64(record.data()));
    const bool accepted = guard.accept(record.data() + 8, wheel::stream_packet_size, now, output);
    const auto* expired = guard.expired(now);
    if (!expired) output = guard.sample(output, .01);
    std::cout << accepted << " " << guard.sequence << " " << guard.goal[0] << " "
              << output[0] << " " << (expired ? expired : "active") << "\n";
  }
}
