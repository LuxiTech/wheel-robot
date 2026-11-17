#!/usr/bin/env python3
"""Build visual servo candidates from actual SDK sources without installing."""

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
    folder = Path(__file__).parent
    text = replace_once(
        text,
        '  std::string joint_phase_ = "idle",',
        (folder / "adapter_members.inc").read_text() + '\n  std::string joint_phase_ = "idle",',
    )
    text = replace_once(
        text,
        'return joint_phase_ == "preparing" || joint_phase_ == "moving";',
        'return joint_phase_ == "preparing" || joint_phase_ == "moving" || joint_phase_ == "servoing";',
    )
    text = replace_once(
        text,
        "    if (!joint_active()) return;",
        '    if (!joint_active()) return;\n    if (joint_phase_ == "servoing") { update_visual(t, dt); return; }',
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
        '    if (command == "visual_servo_start" || command == "visual_servo_update") return visual_command(command, detail, message);\n    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);',
    )
    text = replace_once(
        text,
        "    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { update(); });",
        """    visual_targets_ = create_publisher<std_msgs::msg::String>("/wheel/visual_servo/target", 1);
    visual_intents_ = create_subscription<std_msgs::msg::String>("/wheel/visual_servo/intent", 1,
        [this](std_msgs::msg::String::ConstSharedPtr packet) { accept_visual_intent(packet->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { update(); });""",
    )
    text = replace_once(
        text,
        '(!joint_active() || (joint_phase_ == "moving" && joint_progress_ >= 1.))',
        '(!joint_active() || joint_phase_ == "servoing" || (joint_phase_ == "moving" && joint_progress_ >= 1.))',
    )
    text = replace_once(
        text,
        '{"joint_control", {{"phase", joint_phase_},',
        """{"joint_control", {{"visual_servo_protocol", 1}, {"visual_sequence", visual_sequence_},
                           {"visual_image_receive_mono_ns", visual_image_ns_}, {"visual_intent_mono_ns", visual_intent_ns_},
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
    (output / "ik_worker.cpp").write_bytes(Path(__file__).with_name("ik_worker.cpp").read_bytes())
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
        raise ValueError("Unique live SDK OpenArm library required")
    cmake = f'''cmake_minimum_required(VERSION 3.16)
project(wheel_visual_servo_candidate)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -DNDEBUG")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(luxi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(luxi_core REQUIRED)
find_package(Threads REQUIRED)
add_executable(openarm_adapter_visual_candidate openarm_adapter.cpp)
target_include_directories(openarm_adapter_visual_candidate PRIVATE "{root}/Project/device_adapters/src" "{root}/Project/device_adapters/include" "{root}/device/openarm/openarm_can/include")
target_link_libraries(openarm_adapter_visual_candidate "{libraries[0]}" Threads::Threads)
ament_target_dependencies(openarm_adapter_visual_candidate rclcpp luxi_interfaces luxi_core std_msgs)
add_executable(wheel_visual_ik_candidate ik_worker.cpp)
target_include_directories(wheel_visual_ik_candidate PRIVATE "{root}/Project/arm_motion/include" "{root}/3part/external/mujoco/include")
target_link_libraries(wheel_visual_ik_candidate "{root}/3part/external/mujoco/lib/libmujoco.so")
ament_target_dependencies(wheel_visual_ik_candidate rclcpp luxi_interfaces luxi_core std_msgs)
'''
    (output / "CMakeLists.txt").write_text(cmake)
    resources = dict(
        adapter=source,
        ik=root / "Project/arm_motion/include/luxi/ik.hpp",
        model=root
        / "build/install/luxi_description/share/luxi_description/model/openarm_claw14.xml",
    )
    report = dict(
        sources={
            name: dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            for name, path in resources.items()
        },
        candidate_sha256=hashlib.sha256(revised.encode()).hexdigest(),
        installed=False,
        started=False,
        live_files_modified=False,
    )
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
