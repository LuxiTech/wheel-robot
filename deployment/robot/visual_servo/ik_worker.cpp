// Reuse the deployed SDK's IK and pose filter; this process never opens CAN.
#include <luxi/ik.hpp>
#include <std_msgs/msg/string.hpp>

class VisualIK : public rclcpp::Node {
  luxi::ArmConfig cfg_;
  luxi::Ik ik_;
  luxi::Json input_;
  luxi::Targets targets_;
  std::array<double, 16> reference_{};
  std::array<luxi::PoseFilter, 2> filters_;
  std::string session_;
  int64_t image_ns_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

 public:
  VisualIK() : Node("wheel_visual_ik"),
      cfg_(luxi::config(declare_parameter<std::string>("config", ""))),
      ik_(declare_parameter<std::string>("model", ""), cfg_) {
    for (auto& filter : filters_) filter = luxi::PoseFilter(4.0);
    // Keep the SDK model, limits, posture regularizer and solver unchanged.
    output_ = create_publisher<std_msgs::msg::String>("/wheel/visual_servo/intent", 1);
    input_sub_ = create_subscription<std_msgs::msg::String>(
        "/wheel/visual_servo/target", 1, [this](std_msgs::msg::String::ConstSharedPtr message) {
      try {
        const auto next = luxi::Json::parse(message->data);
        const auto stamp = next.at("image_receive_mono_ns").get<int64_t>();
        const auto session = next.at("request_id").get<std::string>();
        if (session != session_) {
          reference_ = next.at("seed").get<std::array<double, 16>>();
          session_ = session;
          image_ns_ = 0;
          for (auto& filter : filters_) filter.reset();
        }
        if (stamp <= image_ns_) return;
        for (int side = 0; side < 2; ++side) {
          const auto rows = next.at("targets").at(side ? "left" : "right");
          Eigen::Vector3d p;
          Eigen::Matrix3d r;
          for (int row = 0; row < 3; ++row) {
            p[row] = rows.at(row).at(3).get<double>();
            for (int col = 0; col < 3; ++col) r(row, col) = rows.at(row).at(col).get<double>();
          }
          auto filtered = filters_[side].update(luxi::pose(p, Eigen::Quaterniond(r)), stamp);
          if (side) targets_.left = filtered;
          else targets_.right = filtered;
        }
        targets_.grippers = next.at("grippers").get<std::array<double, 2>>();
        input_ = next;
        image_ns_ = stamp;
      } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Rejected visual target: %s", error.what());
      }
    });
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this] {
      const auto now_ns = luxi::mono();
      if (!image_ns_ || now_ns < image_ns_ || now_ns - image_ns_ > 500000000) return;
      try {
        // Integrate the commanded reference as in the live arm_motion process;
        // do not repeatedly re-seed from gravity-deflected measured angles.
        reference_ = ik_.solve(targets_, reference_);
        std_msgs::msg::String out;
        out.data = luxi::Json{{"request_id", session_}, {"sequence", input_.at("sequence")},
            {"image_receive_mono_ns", image_ns_}, {"emitted_mono_ns", luxi::mono()},
            {"positions", reference_}}.dump();
        output_->publish(out);
      } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Visual IK: %s", error.what());
      }
    });
  }
};

int main(int argc, char** argv) {
  if (argc == 5 && std::string(argv[1]) == "--fk") {
    luxi::Ik ik(argv[3], luxi::ArmConfig(luxi::config(argv[2])));
    ik.sync(luxi::Json::parse(argv[4]).get<std::array<double, 16>>());
    luxi::Json result;
    for (int side = 0; side < 2; ++side) {
      const auto value = ik.fk(side);
      const auto r = luxi::rot(value).toRotationMatrix();
      const auto p = luxi::pos(value);
      std::array<std::array<double, 4>, 4> rows{};
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) rows[row][col] = r(row, col);
        rows[row][3] = p[row];
      }
      rows[3][3] = 1;
      result[side ? "left" : "right"] = rows;
    }
    std::cout << result.dump() << std::endl;
    return 0;
  }
  return luxi::node_main(argc, argv, [] { return std::make_shared<VisualIK>(); });
}
