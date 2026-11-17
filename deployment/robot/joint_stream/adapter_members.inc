  wheel::JointStreamGuard stream_guard_;
  wheel::JointStreamSocket stream_socket_;
  int stream_port_ = 18034;

  bool stream_console(int64_t now) const {
    return luxi::fresh(control_status_time_, now, 500000000) &&
        control_status_.value("control_source", "") == "console" &&
        (control_status_.value("system_phase", "disabled") == "disabled" ||
         control_status_.value("system_phase", "disabled") == "stopped");
  }

  bool stream_command(const std::string& detail, std::string& message) {
    try {
      const auto packet = luxi::Json::parse(detail);
      const auto id = packet.at("request_id").get<std::string>();
      const auto key = packet.at("stream_key").get<std::string>();
      const double speed = packet.value("speed_rad_s", .2);
      const double acceleration = packet.value("acceleration_rad_s2", .5);
      const auto now = luxi::mono();
      if (id.empty() || id.size() > 96 || !enabled_ || manual_fault_latched_ ||
          joint_phase_ == "fault" || !stream_console(now))
        throw std::invalid_argument("Owned enabled console hold required; no implicit reset or enable");
      wheel::Axes speeds = cfg_.speed;
      for (auto& value : speeds) value = std::min(value, speed);
      wheel::JointStreamGuard candidate;
      candidate.begin(key, cfg_.lo, cfg_.hi, speeds, nominal_sent_, now, acceleration);
      if (!joint_command("joint_move", luxi::Json{{"request_id", id}, {"unit", "rad"},
          {"positions", nominal_sent_}, {"speed_rad_s", speed}}.dump(), message)) return false;
      if (joint_id_ != id || joint_phase_ != "preparing")
        throw std::invalid_argument("Stream identity already consumed");
      stream_guard_ = std::move(candidate);
      joint_phase_ = "streaming"; joint_reason_ = "waiting_for_laptop_joint_reference";
      message = "Laptop stream active; joint reference, feedback and image watchdogs enabled";
      return true;
    } catch (const std::exception& error) { message = error.what(); return false; }
  }

  void update_stream(int64_t now, double dt) {
    if (!manual_feedback_fresh(now) || !joint_positions_in_range(state_.position)) {
      finish_joint("fault", "joint_stream_feedback_invalid", true); return;
    }
    if (!stream_console(now)) { finish_joint("stopped", "joint_stream_authority_lost"); return; }
    stream_socket_.drain(stream_guard_, now, nominal_sent_);
    if (const auto* reason = stream_guard_.expired(now)) {
      finish_joint("stopped", reason); return;
    }
    for (int axis = 0; axis < 16; ++axis)
      if (std::abs(sent_[axis] - state_.position[axis]) > .25) {
        finish_joint("fault", "joint_stream_following_error"); return;
      }
    try {
      nominal_sent_ = stream_guard_.sample(nominal_sent_, dt);
    } catch (const std::exception&) {
      finish_joint("stopped", "joint_stream_board_reference_rejected"); return;
    }
    joint_target_ = stream_guard_.goal;
    command_.position = nominal_sent_; command_.phase = "joint_stream";
    command_.source_mono_ns = now;
    joint_reason_ = stream_guard_.sequence ? "following_laptop_joint_references" : "waiting_for_laptop_joint_reference";
  }
