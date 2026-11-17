#!/usr/bin/env python3
"""Build a separate receiver from current robot SDK sources. Never install/start."""

import argparse
import difflib
import hashlib
import json
from pathlib import Path


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError("Live SDK anchor mismatch: " + before[:100])
    return text.replace(before, after, 1)


def patch_adapter(text):
    result = _patch_adapter(text)
    if '"joint_stream_fixed_target_protocol"' not in result:
        result = replace_once(
            result,
            '{"joint_stream_protocol", 1}',
            '{"joint_stream_protocol", 1}, {"joint_stream_fixed_target_protocol", 1}',
        )
    if '"joint_stream_planned_trajectory_protocol"' not in result:
        result = replace_once(
            result,
            '{"joint_stream_fixed_target_protocol", 1}',
            '{"joint_stream_fixed_target_protocol", 1}, {"joint_stream_planned_trajectory_protocol", 1}',
        )
    return result


def _patch_adapter(text):
    folder = Path(__file__).parent
    if hashlib.sha256(text.encode()).hexdigest() == (
        "7658e5144914f1f1ac3cb0458e1693df9d1981c66e5f295190cbb62066503297"
    ):
        return text
    if '"joint_stream_protocol"' in text:
        if hashlib.sha256(text.encode()).hexdigest() == (
            "1bdf342a302d95c23100a069900e0d29f1ce469a15945c8bed54eea3c439aebc"
        ):
            return replace_once(
                text,
                '{"joint_stream_protocol", 1}',
                '{"joint_stream_protocol", 1}, {"joint_stream_fixed_target_protocol", 1}',
            )
        if hashlib.sha256(text.encode()).hexdigest() != (
            "8724fdde867f6eda4471c468a0d5aaf70f93726808b6ba2e00a586ce40bd6db5"
        ):
            raise ValueError(
                "Unknown installed stream adapter; inspect its changes before upgrading"
            )
        return replace_once(
            text,
            '{"joint_stream_protocol", 1}, {"joint_stream_port", stream_port_},',
            '{"joint_stream_protocol", 1}, {"joint_stream_port", stream_port_},\n'
            '                           {"joint_stream_image_max_age_ms", '
            "wheel::stream_image_age_ns / 1000000},",
        )
    text = replace_once(
        text,
        '#include "static_position_compensation.hpp"',
        '#include "static_position_compensation.hpp"\n#include "joint_stream.hpp"',
    )
    text = replace_once(
        text,
        '  std::string joint_phase_ = "idle",',
        (folder / "adapter_members.inc").read_text() + '\n  std::string joint_phase_ = "idle",',
    )
    text = replace_once(
        text,
        'return joint_phase_ == "preparing" || joint_phase_ == "moving";',
        'return joint_phase_ == "preparing" || joint_phase_ == "moving" || joint_phase_ == "streaming";',
    )
    text = replace_once(
        text,
        "    if (!joint_active()) return;",
        '    if (!joint_active()) return;\n    if (joint_phase_ == "streaming") { update_stream(t, dt); return; }',
    )
    text = replace_once(
        text,
        '    if (command == "joint_stop") {',
        """    if (command == "joint_stop") {
      if (!detail.empty()) {
        try {
          if (luxi::Json::parse(detail).at("request_id").get<std::string>() != joint_id_) {
            message = "Unowned joint stop rejected"; return false;
          }
        } catch (const std::exception& error) { message = error.what(); return false; }
      }""",
    )
    text = replace_once(
        text,
        '    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);',
        '    if (command == "joint_stream_start") return stream_command(detail, message);\n'
        '    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);',
    )
    text = replace_once(
        text,
        "    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { update(); });",
        """    stream_port_ = declare_parameter<int>("joint_stream_port", 18034);
    stream_socket_.open(declare_parameter<std::string>("joint_stream_bind", "127.0.0.1"), stream_port_);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { update(); });""",
    )
    text = replace_once(
        text,
        '(!joint_active() || (joint_phase_ == "moving" && joint_progress_ >= 1.))',
        '(!joint_active() || joint_phase_ == "streaming" || (joint_phase_ == "moving" && joint_progress_ >= 1.))',
    )
    text = replace_once(
        text,
        '{"joint_control", {{"phase", joint_phase_},',
        """{"joint_control", {{"joint_stream_protocol", 1}, {"joint_stream_port", stream_port_},
                           {"joint_stream_image_max_age_ms", wheel::stream_image_age_ns / 1000000},
                           {"joint_stream_speed_limits", cfg_.speed},
                           {"joint_stream_sequence", stream_guard_.sequence},
                           {"phase", joint_phase_},""",
    )
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.sdk_root.resolve(), args.output.resolve()
    original_path = root / "Project/device_adapters/src/openarm_adapter.cpp"
    original_bytes = original_path.read_bytes()
    original = original_path.read_text()
    revised = patch_adapter(original)
    libraries = list((root / "build").rglob("libluxi_openarm_can.a"))
    if len(libraries) != 1:
        raise ValueError("Unique actual SDK OpenArm library required")
    output.mkdir(parents=True, exist_ok=False)
    (output / "openarm_adapter.cpp").write_text(revised)
    (output / "joint_stream.hpp").write_bytes(
        Path(__file__).with_name("joint_stream.hpp").read_bytes()
    )
    (output / "adapter.patch").write_text(
        "".join(
            difflib.unified_diff(
                original.splitlines(True),
                revised.splitlines(True),
                fromfile=str(original_path),
                tofile="candidate/openarm_adapter.cpp",
            )
        )
    )
    cmake = f'''cmake_minimum_required(VERSION 3.16)
project(wheel_joint_stream_candidate)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luxi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(luxi_core REQUIRED)
find_package(Threads REQUIRED)
add_executable(openarm_adapter_stream_candidate openarm_adapter.cpp)
target_include_directories(openarm_adapter_stream_candidate PRIVATE "{root}/Project/device_adapters/src" "{root}/Project/device_adapters/include" "{root}/device/openarm/openarm_can/include")
target_link_libraries(openarm_adapter_stream_candidate "{libraries[0]}" Threads::Threads)
ament_target_dependencies(openarm_adapter_stream_candidate rclcpp luxi_interfaces luxi_core std_msgs)
'''
    (output / "CMakeLists.txt").write_text(cmake)
    report = dict(
        source=str(original_path),
        source_sha256=hashlib.sha256(original_bytes).hexdigest(),
        source_normalized_sha256=hashlib.sha256(original.encode()).hexdigest(),
        candidate_sha256=hashlib.sha256(revised.encode()).hexdigest(),
        installed=False,
        started=False,
        live_files_modified=False,
    )
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
