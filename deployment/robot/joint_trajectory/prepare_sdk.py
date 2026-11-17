#!/usr/bin/env python3
"""Prepare/build a separate adapter from live SDK sources; never install/start it."""

import argparse
import difflib
import hashlib
import json
from pathlib import Path


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError("Live SDK anchor mismatch: " + before[:100])
    return text.replace(before, after, 1)


TRAJECTORY_COMMAND = r"""
  bool trajectory_command(const std::string& detail, std::string& message) {
    try {
      const auto j = luxi::Json::parse(detail);
      if (!j.is_object() || !j.contains("waypoints") || !j.at("waypoints").is_array() ||
          j.at("waypoints").size() < 2 || j.at("waypoints").size() > 4096 ||
          !j.contains("request_id") || !j.at("request_id").is_string() ||
          j.value("unit", std::string("rad")) != "rad")
        throw std::invalid_argument("Trajectory requires 2..4096 sixteen-axis radian waypoints");
      const auto id = j.at("request_id").get<std::string>();
      const double speed = j.value("speed_rad_s", .2);
      const double acceleration = j.value("acceleration_rad_s2", .5);
      const double settle = j.value("settle_timeout_s", 2.);
      if (id.empty() || id.size() > 96 || !std::isfinite(settle) || settle < .5 || settle > 10.)
        throw std::invalid_argument("Invalid trajectory request identity/settle timeout");
      std::vector<luxi::JointPath::Axes> points;
      for (const auto& row : j.at("waypoints")) {
        if (!row.is_array() || row.size() != 16)
          throw std::invalid_argument("Each trajectory waypoint needs sixteen angles");
        luxi::JointPath::Axes point{};
        for (int axis = 0; axis < 16; ++axis) {
          if (!row.at(axis).is_number()) throw std::invalid_argument("Non-numeric waypoint");
          point[axis] = row.at(axis).get<double>();
        }
        points.push_back(point);
      }
      // Build and validate the entire path before ownership or motor mutations.
      luxi::JointPath validation(points, cfg_.lo, cfg_.hi, cfg_.speed, speed, acceleration);
      const auto fingerprint = luxi::Json{{"kind", "joint_trajectory"}, {"waypoints", points},
          {"speed_rad_s", speed}, {"acceleration_rad_s2", acceleration},
          {"settle_timeout_s", settle}}.dump();
      const auto prior = joint_requests_.find(id);
      if (prior != joint_requests_.end()) {
        message = prior->second == fingerprint ? "Trajectory already accepted" : "Request ID content mismatch";
        return prior->second == fingerprint;
      }
      if (!enabled_ || manual_fault_latched_)
        throw std::invalid_argument("An already-enabled, non-faulted hold is required");
      if (!manual_feedback_rejection(luxi::mono()).empty())
        throw std::invalid_argument("Fresh stationary feedback in original limits required");
      for (int axis = 0; axis < 16; ++axis) {
        if (axis % 8 != 7 && std::abs(points.front()[axis] - state_.position[axis]) > .015)
          throw std::invalid_argument("Trajectory start no longer matches measured joints");
        // Motion phases retain the existing gripper hold. Opening/closure have
        // their own SDK requests and load checks outside a continuous arm phase.
        if (axis % 8 == 7)
          for (const auto& point : points)
            if (std::abs(point[axis] - nominal_sent_[axis]) > 1e-8)
              throw std::invalid_argument("Gripper must retain its hold during an arm trajectory");
      }
      const auto final = points.back();
      points.insert(points.begin(), nominal_sent_);
      luxi::JointPath candidate(points, cfg_.lo, cfg_.hi, cfg_.speed, speed, acceleration);
      const auto endpoint = luxi::Json{{"request_id", id}, {"positions", final},
          {"unit", "rad"}, {"speed_rad_s", speed}}.dump();
      if (!joint_command("joint_move", endpoint, message)) return false;
      joint_path_.emplace(std::move(candidate));
      joint_duration_ = joint_path_->duration();
      joint_settle_timeout_ = settle;
      joint_requests_[id] = fingerprint;
      message = "Complete trajectory accepted; intermediate waypoints do not wait for arrival";
      return true;
    } catch (const std::exception& error) {
      message = error.what();
      return false;
    }
  }
"""


def patch_adapter(text):
    text = replace_once(
        text,
        '#include "static_position_compensation.hpp"',
        '#include "static_position_compensation.hpp"\n#include "joint_path.hpp"\n#include <optional>',
    )
    text = replace_once(
        text,
        '  std::string joint_phase_ = "idle", joint_reason_ = "not_requested", joint_id_;',
        '  std::optional<luxi::JointPath> joint_path_;\n  double joint_settle_timeout_ = 2.;\n  std::string joint_phase_ = "idle", joint_reason_ = "not_requested", joint_id_;',
    )
    text = replace_once(
        text,
        "  bool joint_command(const std::string& command, const std::string& detail, std::string& message) {",
        TRAJECTORY_COMMAND
        + "\n  bool joint_command(const std::string& command, const std::string& detail, std::string& message) {",
    )
    text = replace_once(
        text,
        "    writable_buses();\n    startup_owned_ = startup_pending_ = restore_hold_ = false;",
        "    writable_buses();\n    joint_path_.reset();\n    startup_owned_ = startup_pending_ = restore_hold_ = false;",
    )
    text = replace_once(
        text,
        '    const bool console_action = command == "joint_move" || command == "start" ||',
        '    const bool console_action = command == "joint_trajectory" || command == "joint_move" || command == "start" ||',
    )
    text = replace_once(
        text,
        '    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);',
        '    if (command == "joint_trajectory") return trajectory_command(detail, message);\n    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);',
    )
    text = replace_once(
        text,
        '    if (command == "joint_stop") {',
        """    if (command == "joint_stop") {
      if (!detail.empty()) {
        try {
          const auto identity = luxi::Json::parse(detail).at("request_id").get<std::string>();
          if (identity != joint_id_) { message = "Unowned joint stop rejected"; return false; }
        } catch (const std::exception& error) { message = error.what(); return false; }
      }""",
    )
    text = replace_once(
        text,
        '      if ((t - joint_since_) * 1e-9 > joint_duration_ * 2 + 10) { finish_joint("fault", "joint_motion_timeout"); return; }',
        """      if (joint_path_ && (t - joint_since_) * 1e-9 > joint_duration_ + joint_settle_timeout_) {
        finish_joint("needs_correction", "joint_trajectory_endpoint_not_reached"); return;
      }
      if ((t - joint_since_) * 1e-9 > joint_duration_ * 2 + 10) { finish_joint("fault", "joint_motion_timeout"); return; }""",
    )
    text = replace_once(
        text,
        "      bool arrived = joint_progress_ >= 1.;\n      for (int i = 0; i < 16; ++i) {",
        "      bool arrived = joint_progress_ >= 1.;\n      const auto path_target = joint_path_ ? joint_path_->sample(joint_progress_ * joint_duration_) : joint_target_;\n      for (int i = 0; i < 16; ++i) {",
    )
    text = replace_once(
        text,
        """        nominal_sent_[i] = joint_progress_ >= 1. ? joint_target_[i] :
            std::clamp(joint_from_[i] + (joint_target_[i] - joint_from_[i]) * joint_progress_, cfg_.lo[i], cfg_.hi[i]);""",
        """        nominal_sent_[i] = joint_path_ ? path_target[i] : (joint_progress_ >= 1. ? joint_target_[i] :
            std::clamp(joint_from_[i] + (joint_target_[i] - joint_from_[i]) * joint_progress_, cfg_.lo[i], cfg_.hi[i]));""",
    )
    text = replace_once(
        text,
        '{"joint_control", {{"phase", joint_phase_},',
        """{"joint_control", {{"trajectory_protocol", 1},
                           {"trajectory_points", joint_path_ ? joint_path_->size() : 0},
                           {"trajectory_duration_s", joint_duration_}, {"trajectory_fraction", joint_progress_},
                           {"phase", joint_phase_},""",
    )
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.sdk_root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = root / "Project/device_adapters/src/openarm_adapter.cpp"
    original = source.read_text()
    revised = patch_adapter(original)
    (output / "openarm_adapter.cpp").write_text(revised)
    (output / "joint_path.hpp").write_bytes(Path(__file__).with_name("joint_path.hpp").read_bytes())
    (output / "adapter.patch").write_text(
        "".join(
            difflib.unified_diff(
                original.splitlines(True),
                revised.splitlines(True),
                fromfile=str(source),
                tofile="candidate/openarm_adapter.cpp",
            )
        )
    )
    libraries = list((root / "build").rglob("libluxi_openarm_can.a"))
    if len(libraries) != 1:
        raise ValueError("Expected exactly one live SDK OpenArm static library")
    cmake = f'''cmake_minimum_required(VERSION 3.16)
project(wheel_joint_trajectory_candidate)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luxi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(luxi_core REQUIRED)
find_package(Threads REQUIRED)
add_executable(openarm_adapter_trajectory_candidate openarm_adapter.cpp)
target_include_directories(openarm_adapter_trajectory_candidate PRIVATE
  "{root}/Project/device_adapters/src" "{root}/Project/device_adapters/include"
  "{root}/device/openarm/openarm_can/include")
target_link_libraries(openarm_adapter_trajectory_candidate "{libraries[0]}" Threads::Threads)
ament_target_dependencies(openarm_adapter_trajectory_candidate rclcpp luxi_interfaces luxi_core std_msgs)
'''
    (output / "CMakeLists.txt").write_text(cmake)
    report = dict(
        source=str(source),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        candidate_sha256=hashlib.sha256(revised.encode()).hexdigest(),
        header_sha256=hashlib.sha256((output / "joint_path.hpp").read_bytes()).hexdigest(),
        live_files_modified=False,
        installed=False,
        started=False,
    )
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
