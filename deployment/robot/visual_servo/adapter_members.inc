  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visual_targets_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_intents_;
  uint64_t visual_sequence_ = 0;
  int64_t visual_image_ns_ = 0, visual_intent_ns_ = 0;
  std::string visual_pair_;
  std::array<double, 16> visual_goal_{}, visual_velocity_{};
  static constexpr int64_t visual_age_ns_ = 500000000;

  bool visual_console(int64_t now) const {
    return luxi::fresh(control_status_time_, now, 500000000) &&
        control_status_.value("control_source", "") == "console" &&
        (control_status_.value("system_phase", "disabled") == "disabled" ||
         control_status_.value("system_phase", "disabled") == "stopped");
  }

  bool visual_command(const std::string& command, const std::string& detail, std::string& message) {
    try {
      const auto packet = luxi::Json::parse(detail);
      const auto id = packet.at("request_id").get<std::string>();
      const auto now = luxi::mono();
      if (id.empty() || id.size() > 96) throw std::invalid_argument("Invalid visual session identity");
      if (command == "visual_servo_start") {
        if (joint_phase_ == "servoing" && joint_id_ == id) { message = "Visual session already active"; return true; }
        if (!enabled_ || manual_fault_latched_ || joint_phase_ == "fault" || !visual_console(now))
          throw std::invalid_argument("Enabled non-faulted console hold required; no implicit reset or enable");
        const double speed = packet.value("speed_rad_s", .2);
        if (!joint_command("joint_move", luxi::Json{{"request_id", id}, {"unit", "rad"},
            {"positions", nominal_sent_}, {"speed_rad_s", speed}}.dump(), message)) return false;
        if (joint_id_ != id || joint_phase_ != "preparing")
          throw std::invalid_argument("Visual session ID was previously consumed");
        joint_phase_ = "servoing"; joint_reason_ = "waiting_for_visual_target";
        visual_sequence_ = 0; visual_pair_.clear(); visual_image_ns_ = visual_intent_ns_ = 0;
        visual_goal_ = nominal_sent_; visual_velocity_.fill(0);
        joint_since_ = now;
        message = "Visual servo session active; fresh images and IK output required";
        return true;
      }
      if (command != "visual_servo_update" || joint_phase_ != "servoing" || joint_id_ != id || !visual_console(now))
        throw std::invalid_argument("No owned active console visual session");
      if (!packet.at("sequence").is_number_unsigned() && !packet.at("sequence").is_number_integer())
        throw std::invalid_argument("Integer visual sequence required");
      const auto sequence = packet.at("sequence").get<int64_t>();
      const auto stamp = packet.at("image_receive_mono_ns").get<int64_t>();
      const auto pair = packet.at("pair_id").get<std::string>();
      if (sequence <= 0 || static_cast<uint64_t>(sequence) <= visual_sequence_ || pair.empty() ||
          pair == visual_pair_ || stamp <= visual_image_ns_ || stamp > now || now - stamp > visual_age_ns_ ||
          packet.value("frame_id", "") != "arm_origin")
        throw std::invalid_argument("Fresh distinct visual frame, increasing sequence and arm_origin required");
      for (const auto* side : {"left", "right"}) {
        const auto& rows = packet.at("targets").at(side);
        if (!rows.is_array() || rows.size() != 4) throw std::invalid_argument("Rigid 4x4 targets required");
        Eigen::Matrix4d matrix;
        for (int row = 0; row < 4; ++row) {
          if (!rows.at(row).is_array() || rows.at(row).size() != 4) throw std::invalid_argument("Rigid 4x4 targets required");
          for (int col = 0; col < 4; ++col) {
            if (!rows.at(row).at(col).is_number()) throw std::invalid_argument("Numeric target required");
            matrix(row, col) = rows.at(row).at(col).get<double>();
          }
        }
        const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
        if (!matrix.allFinite() || (matrix.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-6 ||
            (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-4 ||
            std::abs(rotation.determinant() - 1.) > 1e-4)
          throw std::invalid_argument("Invalid rigid target transform");
      }
      auto accepted = packet;
      accepted["seed"] = nominal_sent_;
      accepted["grippers"] = std::array<double, 2>{nominal_sent_[7], nominal_sent_[15]};
      std_msgs::msg::String target;
      target.data = accepted.dump();
      visual_sequence_ = static_cast<uint64_t>(sequence); visual_image_ns_ = stamp; visual_pair_ = pair;
      visual_targets_->publish(target);
      message = "New visual target accepted without stopping or replanning a path";
      return true;
    } catch (const std::exception& error) { message = error.what(); return false; }
  }

  void accept_visual_intent(const std::string& text) {
    try {
      const auto packet = luxi::Json::parse(text);
      const auto now = luxi::mono();
      const auto stamp = packet.at("image_receive_mono_ns").get<int64_t>();
      const auto emitted = packet.at("emitted_mono_ns").get<int64_t>();
      if (joint_phase_ != "servoing" || packet.at("request_id").get<std::string>() != joint_id_ ||
          packet.at("sequence").get<uint64_t>() != visual_sequence_ || stamp != visual_image_ns_ ||
          stamp > now || now - stamp > visual_age_ns_ || emitted > now || now - emitted > 150000000 ||
          emitted <= visual_intent_ns_) return;
      const auto positions = packet.at("positions").get<std::array<double, 16>>();
      if (!joint_positions_in_range(positions) || std::abs(positions[7] - nominal_sent_[7]) > 1e-8 ||
          std::abs(positions[15] - nominal_sent_[15]) > 1e-8) return;
      visual_goal_ = positions; visual_intent_ns_ = emitted;
    } catch (const std::exception& error) { error_visual_rejection_ = error.what(); }
  }
  std::string error_visual_rejection_;

  void update_visual(int64_t now, double dt) {
    if (!manual_feedback_fresh(now) || !joint_positions_in_range(state_.position)) {
      finish_joint("fault", "visual_feedback_invalid", true); return;
    }
    if (!visual_console(now)) { finish_joint("stopped", "visual_authority_lost"); return; }
    if ((!visual_image_ns_ && now - joint_since_ > visual_age_ns_) ||
        (visual_image_ns_ && now - visual_image_ns_ > visual_age_ns_)) {
      finish_joint("stopped", "visual_target_timeout"); return;
    }
    if ((!visual_intent_ns_ && now - joint_since_ > visual_age_ns_) ||
        (visual_intent_ns_ && now - visual_intent_ns_ > 150000000)) {
      finish_joint("stopped", "visual_ik_timeout"); return;
    }
    for (int axis = 0; axis < 16; ++axis) {
      if (std::abs(sent_[axis] - state_.position[axis]) > .25) {
        finish_joint("fault", "visual_following_error"); return;
      }
      if (axis % 8 == 7) continue;
      const double error = visual_goal_[axis] - nominal_sent_[axis];
      const double acceleration = std::clamp(64. * error - 16. * visual_velocity_[axis], -.5, .5);
      const double speed = std::min(joint_speed_, cfg_.speed[axis]);
      visual_velocity_[axis] = std::clamp(visual_velocity_[axis] + acceleration * dt, -speed, speed);
      const double candidate = nominal_sent_[axis] + visual_velocity_[axis] * dt;
      if (candidate < cfg_.lo[axis] || candidate > cfg_.hi[axis]) {
        finish_joint("fault", "visual_reference_limit"); return;
      }
      nominal_sent_[axis] = candidate;
    }
    joint_target_ = visual_goal_;
    command_.position = nominal_sent_; command_.phase = "visual_servo"; command_.source_mono_ns = now;
    joint_reason_ = "following_live_visual_targets";
  }
