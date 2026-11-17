// The controller manager runs on the robot. No HTTP, IK or file IO in read/write.
// The SDK is the sole CAN owner; lifecycle activation never enables motors.
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <luxi_interfaces/msg/arm_state.hpp>
#include <luxi_interfaces/srv/command.hpp>
#include <nlohmann/json.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <random>
#include <thread>

namespace wheel_moveit_control {
using Json = nlohmann::json;
using Return = hardware_interface::return_type;
using Callback = hardware_interface::CallbackReturn;
static int64_t mono() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
class SDKSystem : public hardware_interface::SystemInterface {
  std::array<double,14> q_{}, v_{}, effort_{}, position_{}, velocity_{};
  std::array<size_t,14> indices_{};
  std::array<double,16> hold_{};
  luxi_interfaces::msg::ArmState state_;
  Json status_;
  int64_t status_at_=0;
  std::mutex mutex_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
  rclcpp::Subscription<luxi_interfaces::msg::ArmState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<luxi_interfaces::srv::Command>::SharedPtr client_;
  int socket_=-1;
  std::string request_, key_, generation_;
  uint64_t sequence_=0, accepted_at_=0, feedback_at_=0;
  bool active_=false;
  bool call(const std::string& command, const Json& detail) {
    try {
    if (!client_->wait_for_service(std::chrono::seconds(2))) return false;
    auto request=std::make_shared<luxi_interfaces::srv::Command::Request>();
    request->command=command; request->detail=detail.dump();
    auto future=client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2))!=std::future_status::ready) return false;
    return future.get()->success;
    } catch (const std::exception& error) {
      if (node_) RCLCPP_ERROR(node_->get_logger(), "SDK lifecycle request failed: %s", error.what());
      return false;
    }
  }
  void stop() {
    active_=false;
    if (!request_.empty() && client_ && !call("joint_stop", {{"request_id",request_}})) {
      RCLCPP_ERROR(node_->get_logger(), "Owned stop was not acknowledged; SDK watchdog remains active");
    }
    if (socket_>=0) {::close(socket_); socket_=-1;}
    request_.clear();
  }
 public:
  ~SDKSystem() override {
    stop(); executor_.cancel();
    if (thread_.joinable()) thread_.join();
  }
  Callback on_init(const hardware_interface::HardwareInfo& info) override {
    if (SystemInterface::on_init(info)!=Callback::SUCCESS || info.joints.size()!=14)
      return Callback::ERROR;
    std::array<bool,16> seen{};
    for (size_t i=0;i<14;++i) {
      indices_[i]=std::stoul(info.joints[i].parameters.at("sdk_index"));
      if(indices_[i]>=16 || indices_[i]%8==7 || seen[indices_[i]]) return Callback::ERROR;
      seen[indices_[i]]=true;
    }
    return Callback::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    for(size_t i=0;i<14;++i) {
      out.emplace_back(info_.joints[i].name,"position",&q_[i]);
      out.emplace_back(info_.joints[i].name,"velocity",&v_[i]);
      out.emplace_back(info_.joints[i].name,"effort",&effort_[i]);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    for(size_t i=0;i<14;++i) {
      out.emplace_back(info_.joints[i].name,"position",&position_[i]);
      out.emplace_back(info_.joints[i].name,"velocity",&velocity_[i]);
    }
    return out;
  }
  Callback on_configure(const rclcpp_lifecycle::State&) override {
    if (thread_.joinable()) return Callback::ERROR;
    node_=std::make_shared<rclcpp::Node>("wheel_sdk_hardware");
    feedback_sub_=node_->create_subscription<luxi_interfaces::msg::ArmState>(
      "/luxi/arms/state",10,[this](const luxi_interfaces::msg::ArmState& value) {
        std::lock_guard<std::mutex> lock(mutex_); state_=value;
      });
    status_sub_=node_->create_subscription<std_msgs::msg::String>(
      "/luxi/arms/startup_status",10,[this](const std_msgs::msg::String& value) {
        try {auto parsed=Json::parse(value.data); std::lock_guard<std::mutex> lock(mutex_);
          status_=std::move(parsed); status_at_=mono();} catch(const std::exception&) {}
      });
    client_=node_->create_client<luxi_interfaces::srv::Command>("/luxi/arms/startup");
    executor_.add_node(node_); thread_=std::thread([this]{executor_.spin();});
    // Do not expose invented zero joint states while waiting for the first SDK sample.
    for (int i=0;i<200;++i) {
      bool fresh = false;
      { std::lock_guard<std::mutex> lock(mutex_);
        fresh = mono()-status_at_ <= 150000000 && state_.receive_mono_ns > 0 &&
                mono()-state_.receive_mono_ns <= 150000000;
      }
      if (fresh && read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)) == Return::OK)
        return Callback::SUCCESS;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return Callback::ERROR;
  }
  Callback on_activate(const rclcpp_lifecycle::State&) override {
    Json status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(mono()-status_at_>150000000 || !state_.enabled || mono()-state_.receive_mono_ns>150000000)
        return Callback::ERROR;
      status=status_;
      if(status["joint_control"].value("joint_stream_planned_trajectory_protocol",0)!=1)
        return Callback::ERROR;
      for(size_t i=0;i<16;++i) {
        if(!state_.valid[i] || std::abs(state_.velocity[i])>=.08) return Callback::ERROR;
        hold_[i]=status["nominal_target"][i];
        if(i%8!=7 && std::abs(hold_[i]-state_.position[i])>.02) return Callback::ERROR;
      }
      for(size_t i=0;i<14;++i) position_[i]=q_[i]=state_.position[indices_[i]];
    }
    velocity_.fill(0);
    std::random_device random;
    key_.clear(); const char* hex="0123456789abcdef";
    for(int i=0;i<32;++i) key_+=hex[random()%16];
    request_="wheel-jtc-"+key_;
    sockaddr_in address{}; address.sin_family=AF_INET;
    address.sin_port=htons(status["joint_control"]["joint_stream_port"].get<int>());
    inet_pton(AF_INET,"127.0.0.1",&address.sin_addr);
    socket_=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(socket_<0 || ::connect(socket_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0) {
      stop(); return Callback::ERROR;
    }
    if(!call("joint_stream_start",{{"request_id",request_},{"stream_key",key_},
        {"speed_rad_s",.2},{"acceleration_rad_s2",.5}})) {stop();return Callback::ERROR;}
    // Source is trajectory-session admission, explicitly NOT an image timestamp.
    accepted_at_=mono();sequence_=0;
    for(int i=0;i<50;++i) {
      { std::lock_guard<std::mutex> lock(mutex_);
        if(status_.contains("joint_control") && status_["joint_control"].value("request_id","")==request_) {
          generation_=status_["control_generation"].get<std::string>();active_=true;return Callback::SUCCESS;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    stop();return Callback::ERROR;
  }
  Callback on_deactivate(const rclcpp_lifecycle::State&) override {stop();return Callback::SUCCESS;}
  Callback on_error(const rclcpp_lifecycle::State&) override {stop();return Callback::SUCCESS;}
  Return read(const rclcpp::Time&,const rclcpp::Duration&) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(mono()-state_.receive_mono_ns>150000000 || mono()-status_at_>150000000) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "SDK feedback stale: state_age_ns=%ld status_age_ns=%ld active=%d",
        mono()-state_.receive_mono_ns, mono()-status_at_, active_);
      // Inactive hardware owns no motion. Keep its last state; the gateway uses
      // independent raw SDK timestamps and will NOT mark this cached state fresh.
      return active_ ? Return::ERROR : Return::OK;
    }
    if(active_ && (!state_.enabled || status_["joint_control"].value("request_id","")!=request_ ||
       status_["joint_control"].value("phase","")!="streaming" ||
       status_["control_generation"].get<std::string>()!=generation_)) return Return::ERROR;
    for(size_t i=0;i<14;++i) {
      size_t k=indices_[i];
      if(!state_.valid[k] || !std::isfinite(state_.position[k]) ||
        !std::isfinite(state_.velocity[k]) || !std::isfinite(state_.effort[k])) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid SDK feedback at index %zu", k);
        return active_ ? Return::ERROR : Return::OK;
      }
      if(active_ && (state_.position[k]<status_["joint_control"]["lower"][k].get<double>() ||
        state_.position[k]>status_["joint_control"]["upper"][k].get<double>())) {
        RCLCPP_ERROR(node_->get_logger(), "SDK measured limit violation: index=%zu q=%.9f", k, state_.position[k]);
        return Return::ERROR;
      }
      q_[i]=state_.position[k];v_[i]=state_.velocity[k];effort_[i]=state_.effort[k];
    }
    feedback_at_=state_.receive_mono_ns;return Return::OK;
  }
  Return write(const rclcpp::Time&,const rclcpp::Duration&) override {
    if(!active_) return Return::OK;
    std::array<unsigned char,336> bytes{};
    std::memcpy(bytes.data(),"WJPLAN1\0",8);std::memcpy(bytes.data()+8,key_.data(),32);
    auto integer=[&bytes](size_t offset,uint64_t value){
      for(int i=7;i>=0;--i){bytes[offset+i]=value&255;value>>=8;}
    };
    integer(40,++sequence_);integer(48,feedback_at_);integer(56,accepted_at_);
    std::memcpy(bytes.data()+64,key_.data(),16);
    auto positions=hold_;std::array<double,16> velocities{};
    for(size_t i=0;i<14;++i) {
      if(!std::isfinite(position_[i]) || !std::isfinite(velocity_[i])) return Return::ERROR;
      positions[indices_[i]]=position_[i];velocities[indices_[i]]=velocity_[i];
    }
    for(size_t i=0;i<16;++i) {
      uint64_t bits;std::memcpy(&bits,&positions[i],8);integer(80+i*8,bits);
      std::memcpy(&bits,&velocities[i],8);integer(208+i*8,bits);
    }
    return ::send(socket_,bytes.data(),bytes.size(),0)==static_cast<ssize_t>(bytes.size()) ? Return::OK : Return::ERROR;
  }
};
}
PLUGINLIB_EXPORT_CLASS(wheel_moveit_control::SDKSystem,hardware_interface::SystemInterface)
