#!/usr/bin/env python3
"""Prepare explicit console/Quest ownership from the live SDK without installing."""

import argparse
import difflib
import hashlib
import json
from pathlib import Path


def replace(text, before, after):
    if text.count(before) != 1:
        raise ValueError(f"Expected one live SDK anchor: {before[:100]!r}")
    return text.replace(before, after, 1)


def patch_manager(text):
    text = replace(
        text,
        '  std::string system_phase_ = "disabled", epoch_;',
        """  bool explicit_source_ = false;
  std::string control_source_ = "console";
  bool startup_motion_active_ = false;
  bool select_source(const std::string& source) {
    if (!explicit_source_) { reason_ = "explicit_control_source_not_enabled"; return false; }
    if (source == control_source_) return true;
    const auto t = luxi::mono();
    if (!arm_feedback(t) || !luxi::fresh(manual_pose_status_ns_, t, 500000000) ||
        !luxi::coordinated_status_fresh(base_status_, t) ||
        !luxi::coordinated_status_fresh(lift_status_, t) ||
        (system_phase_ != "disabled" && system_phase_ != "stopped") ||
        (phase_ != "disabled" && phase_ != "hold") || manual_pose_blocked_ || startup_motion_active_ ||
        base_status_.value("enabled", true) || !base_status_.value("stationary", false) ||
        lift_status_.value("enabled", true) || !lift_status_.value("stationary", false) ||
        lift_status_.value("session_present", true)) {
      reason_ = "control_source_switch_requires_stopped_devices"; return false;
    }
    for (double speed : state_.velocity)
      if (!std::isfinite(speed) || std::abs(speed) > .08) {
        reason_ = "control_source_switch_requires_stopped_arms"; return false;
      }
    // Selection never enables, homes, releases torque, or publishes an arm goal.
    control_source_ = source;
    phase_ = "disabled"; enter("disabled");
    torque_requested_ = rest_ready_ = accepted_goal_valid_ = false;
    intent_ = luxi::Command{};
    clear_manual_start();
    epoch_ = owner_ + ":" + std::to_string(++epoch_count_);
    x_ = quest_.buttons[2]; b_ = quest_.buttons[1];
    x_released_ = x_enable_press_ = false; long_x_ = true;
    tracking_resume_after_ = t;
    reason_ = source == "console" ? "console_control_selected" : "teleop_selected_waiting_for_explicit_start";
    broadcast(); return true;
  }
  std::string system_phase_ = "disabled", epoch_;""",
    )
    text = replace(
        text,
        'm.data = luxi::Json{{"epoch", epoch_}, {"phase", directive()},',
        'm.data = luxi::Json{{"epoch", epoch_}, {"phase", directive()},\n'
        '                         {"control_source", explicit_source_ ? control_source_ : "legacy"},',
    )
    text = replace(
        text,
        "  bool command(const std::string& s) {",
        """  bool command(const std::string& s) {
    if (s == "select_console") return select_source("console");
    if (s == "select_teleop") return select_source("teleop");
    if (explicit_source_ && control_source_ == "console" &&
        (s == "start" || s == "stop" || s == "park")) {
      reason_ = "console_control_selected"; return false;
    }""",
    )
    text = replace(
        text,
        '    shared_ = declare_parameter("shared_enable", false);',
        '    shared_ = declare_parameter("shared_enable", false);\n'
        '    explicit_source_ = declare_parameter("explicit_control_source", false);',
    )
    text = replace(
        text,
        "          if (manual_pose_blocked_) {",
        """          if (explicit_source_ && control_source_ == "console") {
            // Connection, tracking, grips and X never acquire console ownership.
            // B remains an explicit stop; adapters also retain their B stops.
            if (q->buttons[1] && !b_ && manual_pose_phase_ == "moving")
              life("joint_stop:" + manual_pose_generation_);
            x_ = q->buttons[2]; b_ = q->buttons[1];
            x_released_ = x_enable_press_ = false; long_x_ = true;
            return;
          }
          if (manual_pose_blocked_) {""",
    )
    text = replace(
        text,
        "\n    if (manual_pose_blocked_) suspend_for_manual_pose();",
        """
    if (explicit_source_ && control_source_ == "console") {
      // The arm driver keeps its current hold. Do not overwrite console goals.
      phase_ = "disabled"; system_phase_ = "disabled";
      torque_requested_ = rest_ready_ = accepted_goal_valid_ = false;
      reason_ = "console_control_selected";
    } else if (manual_pose_blocked_) suspend_for_manual_pose();""",
    )
    text = replace(
        text,
        '            startup_owned_ = j.value("owned", false);',
        '            startup_owned_ = j.value("owned", false);\n'
        '            startup_motion_active_ = j.value("phase", "idle") == "moving" || j.value("phase", "idle") == "preparing";',
    )
    text = replace(
        text,
        "            if (manual_pose_blocked_) suspend_for_manual_pose();",
        '            if (manual_pose_blocked_ && !(explicit_source_ && control_source_ == "console")) suspend_for_manual_pose();',
    )
    text = replace(
        text,
        '    const auto report_key = system_phase_ + "\\n" + phase_',
        '    const auto report_key = control_source_ + "\\n" + system_phase_ + "\\n" + phase_',
    )
    text = replace(
        text,
        's.data = luxi::Json{{"phase", phase_},',
        's.data = luxi::Json{{"phase", phase_},\n'
        '                        {"control_source", explicit_source_ ? control_source_ : "legacy"},',
    )
    return text


def patch_coordination(text):
    text = replace(
        text,
        "  std::string epoch, phase;",
        '  std::string epoch, phase, control_source = "legacy";',
    )
    return replace(
        text,
        "      epoch = next_epoch; phase = next_phase; sent_ns = stamp;",
        "      epoch = next_epoch; phase = next_phase; sent_ns = stamp;\n"
        '      control_source = j.value("control_source", "legacy");',
    )


def patch_base(text):
    text = replace(
        text,
        "(luxi::fresh(coordination_.sent_ns, t, 250000000) &&\n"
        '         (coordination_.phase == "disabled" || coordination_.phase == "halt"));',
        "(luxi::fresh(coordination_.sent_ns, t, 250000000) &&\n"
        '         coordination_.control_source != "teleop" &&\n'
        '         (coordination_.phase == "disabled" || coordination_.phase == "halt"));',
    )
    return replace(
        text,
        '{"keyboard_takeover_available", keyboard_coordinator_idle(luxi::mono()) && !gate_.enabled},',
        '{"keyboard_takeover_available", keyboard_coordinator_idle(luxi::mono()) && !gate_.enabled},\n'
        '                      {"control_source", coordination_.control_source},',
    )


def patch_lift(text):
    return replace(
        text,
        '              if (op == "arm") {',
        """              if (op != "stop" && coordinated_mode_ &&
                  coordination_.control_source == "teleop")
                throw std::runtime_error("请先在控制台切换为控制台权限");
              if (op == "arm") {""",
    )


def patch_arm(text):
    text = replace(
        text,
        '    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);',
        """    const bool console_action = command == "joint_move" || command == "start" ||
        command == "restore_hold" || command == "pose_enter" || command == "pose_hold";
    if (console_action && control_status_.contains("control_source") &&
        control_status_.value("control_source", "legacy") != "legacy" &&
        (!luxi::fresh(control_status_time_, t, 500000000) ||
         control_status_.value("control_source", "") != "console")) {
      message = "请先在控制台切换为控制台权限"; return false;
    }
    if (command == "joint_move" || command == "joint_stop") return joint_command(command, detail, message);""",
    )
    return replace(
        text,
        "            if (enable_request && luxi::valid_state(state_) &&",
        """            if (enable_request && control_status_.is_object() && control_status_.value("control_source", "legacy") == "console" &&
                luxi::fresh(control_status_time_, requested_at, 500000000)) return;
            if (enable_request && luxi::valid_state(state_) &&""",
    )


def patch_http(text):
    return replace(
        text,
        '      auto client = req.matches[1] == "arm_startup" ? startup_client_',
        """      {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string kind = req.matches[1], command = j["command"];
        const auto source = control_.is_object() ? control_.value("control_source", "legacy") : "legacy";
        const bool select = kind == "control" && (command == "select_console" || command == "select_teleop");
        const bool stop = kind == "control" ? (command == "disable" || command == "halt" || command == "hold") :
            (command == "stop" || command == "disable" || command == "joint_stop" || command == "pose_cancel" || command == "position_stop");
        if (source != "legacy" && !select && !stop && kind != "episode" &&
            (!luxi::fresh(control_time_, luxi::mono(), 500000000) ||
             (kind == "control" ? source != "teleop" : source != "console"))) {
          res.status = 409;
          res.set_content(luxi::Json{{"success", false}, {"message", "请先显式切换控制权限"}}.dump(), "application/json");
          return;
        }
      }
      auto client = req.matches[1] == "arm_startup" ? startup_client_""",
    )


SOURCE_UI = """
let sourceSwitchBusy = false, sourceSwitchError = '';
function consoleSourceReason() {
  const c = state?.control;
  if (!c || c.control_source === 'legacy' || !c.control_source) return '';
  return !c.fresh ? '控制权限状态已过期' : c.control_source !== 'console' ? '请先切换为控制台控制' : '';
}
function controlSourceUI() {
  if (!$('source-console')) return;
  const c = state?.control, ready = connected && state?.system?.services?.control && c?.fresh && ['console','teleop'].includes(c.control_source);
  const consoleSelected = ready && c.control_source === 'console';
  const teleopSelected = ready && c.control_source === 'teleop';
  $('source-console').disabled = !ready || sourceSwitchBusy || consoleSelected;
  $('source-teleop').disabled = !ready || sourceSwitchBusy || teleopSelected;
  $('source-console').setAttribute('aria-pressed', String(consoleSelected));
  $('source-teleop').setAttribute('aria-pressed', String(teleopSelected));
  text('control-source-status', sourceSwitchError || (!ready ? '等待权限协调服务；可在设备采集中开启遥操作程序。' :
    consoleSelected ? '控制台持有权限：可操作关节、底盘和升降。遥操连接、追踪和普通输入不会接管。' :
    '遥操持有权限：仍需显式启用遥操作才会运动。控制台停止按钮始终可用。'));
}
for (const source of ['console','teleop']) {
  const button = $('source-' + source);
  if (button) button.addEventListener('click', async () => {
    if (sourceSwitchBusy) return;
    sourceSwitchBusy = true; sourceSwitchError = ''; controlSourceUI();
    try {
      const result = await request('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:'select_' + source, detail:''})});
      log(result.message); render(await request('/api/state'));
    } catch (error) {
      sourceSwitchError = error.message.includes('control_source_switch_requires_stopped') ?
        '请先停止当前动作并释放底盘、升降会话，再切换；机械臂可保持。' : error.message;
    } finally { sourceSwitchBusy = false; buttons(); }
  });
}
"""


def patch_app(text):
    text = replace(
        text, "function buttons() {", SOURCE_UI + "\nfunction buttons() {\n  controlSourceUI();"
    )
    text = replace(
        text,
        "    let allow = connected && services[kind] && !busy;",
        """    let allow = connected && services[kind] && !busy;
    const sourceReason = consoleSourceReason();
    if (kind === 'arm_startup' && ['start','pose_enter','pose_hold'].includes(cmd) && sourceReason) {
      b.disabled = true; return;
    }
    if (kind === 'control' && ['start','stop','park'].includes(cmd) && s?.control?.control_source === 'console') {
      b.disabled = true; return;
    }""",
    )
    text = replace(
        text,
        "  if (!connected) reason = '网页服务未连接，请检查网络。';",
        "  if (consoleSourceReason()) reason = consoleSourceReason();\n"
        "  else if (!connected) reason = '网页服务未连接，请检查网络。';",
    )
    text = replace(
        text,
        "reason = 'Quest 遥操已占用，请先停止遥操，再打开网页控制。';",
        "reason = '控制协调尚未就绪，请检查遥操作程序服务；仅连接 Quest 不会占用权限。';",
    )
    text = replace(
        text,
        "text('shared-control-status', a?.phase",
        "text('shared-control-status', s.control?.control_source === 'console' ? '控制台持有控制权限，遥操作未启用。' : a?.phase",
    )
    text = replace(
        text,
        ": s.control?.fresh ? (phases[s.control.phase] || s.control.phase) : a?.fresh ?",
        ": s.control?.fresh && s.control.control_source !== 'console' ? (phases[s.control.phase] || s.control.phase) : a?.fresh ?",
    )
    return replace(
        text,
        "  let reason = !connected ? '网页未连接' : !state?.system?.services?.lift ? '升降服务未就绪' :",
        "  let reason = consoleSourceReason() || (!connected ? '网页未连接' : !state?.system?.services?.lift ? '升降服务未就绪' :",
    ).replace(
        "state.control.system_phase) ? '请先停止统一遥操作' : '';",
        "state.control.system_phase) ? '请先停止统一遥操作' : '');",
        1,
    )


def patch_joints(text):
    return replace(
        text,
        "  const c = state?.control;",
        "  if (!reason) reason = consoleSourceReason();\n  const c = state?.control;",
    )


def patch_html(text):
    panel = """<section class="panel" id="control-source-panel" aria-labelledby="control-source-title">
<div class="section-heading"><h2 id="control-source-title">控制权限</h2></div>
<p>显式选择控制台或遥操。连接 Quest 不会自动切换权限；切换本身不会使能、回零或发起运动。</p>
<div class="button-row"><button id="source-console" class="primary" aria-pressed="false" disabled>切换为控制台控制</button><button id="source-teleop" aria-pressed="false" disabled>切换为遥操控制</button></div>
<p id="control-source-status" role="status" aria-live="polite">等待权限协调服务…</p>
<p class="hint">切换前请停止当前动作，并释放底盘和升降会话。机械臂保持不会因切换而失能。</p></section>
"""
    return replace(
        text,
        '<section id="overview" class="devices">',
        panel + '<section id="overview" class="devices">',
    )


def patch_launch(text):
    return replace(
        text,
        '  <node pkg="luxi_control_manager" exec="control_manager" output="screen" respawn="true" respawn_delay="3.0">',
        '  <node pkg="luxi_control_manager" exec="control_manager" output="screen" respawn="true" respawn_delay="3.0">\n'
        '    <param name="explicit_control_source" value="true" />',
    )


PATCHES = {
    "Project/control_manager/src/control_manager.cpp": patch_manager,
    "Project/robot_core/include/luxi/coordinated_control.hpp": patch_coordination,
    "Project/device_adapters/src/r680_adapter.cpp": patch_base,
    "Project/device_adapters/src/lift_adapter.cpp": patch_lift,
    "Project/device_adapters/src/openarm_adapter.cpp": patch_arm,
    "Project/luxi_web_ctrl/src/web_ctrl.cpp": patch_http,
    "Project/luxi_web_ctrl/web/app.js": patch_app,
    "Project/luxi_web_ctrl/web/joint-control.js": patch_joints,
    "Project/luxi_web_ctrl/web/index.html": patch_html,
    "Project/luxi_web_ctrl/scripts/program.launch.xml": patch_launch,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    prepared = []
    for name, patch in PATCHES.items():
        raw = (args.root / name).read_bytes()
        before = raw.decode().replace("\r\n", "\n")
        after = patch(before)
        encoded = after.replace("\n", "\r\n").encode() if b"\r\n" in raw else after.encode()
        prepared.append((name, raw, encoded))
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {}
    changes = []
    for name, before, after in prepared:
        for directory, raw in [("before", before), ("candidate", after)]:
            target = args.output / directory / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(raw)
        manifest[name] = {
            "before_sha256": hashlib.sha256(before).hexdigest(),
            "candidate_sha256": hashlib.sha256(after).hexdigest(),
        }
        changes.extend(
            difflib.unified_diff(
                before.decode().splitlines(True),
                after.decode().splitlines(True),
                fromfile="before/" + name,
                tofile="after/" + name,
            )
        )
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output / "changes.patch").write_text("".join(changes))
    print(args.output.resolve())


if __name__ == "__main__":
    main()
