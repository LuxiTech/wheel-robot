#!/usr/bin/env python3
"""Prepare manual web ownership with a separate 500 ms motion watchdog."""

import argparse
import hashlib
import json
from pathlib import Path

from prepare_sdk import replace_once


def patch_base(text: str) -> str:
    text = replace_once(
        text,
        "  std::string keyboard_owner_;",
        """  std::string keyboard_owner_;
  bool web_keyboard_paused_ = false;
  uint64_t web_keyboard_sequence_ = 0;
  std::string web_keyboard_pause_reason_, last_keyboard_release_reason_;
  static constexpr int64_t web_keyboard_timeout_ns_ = 500000000;
  void pause_web_keyboard(const std::string& reason) {
    if (!web_keyboard_paused_) {
      RCLCPP_WARN(get_logger(), "Web base motion paused, ownership retained: %s", reason.c_str());
      web_keyboard_pause_reason_ = reason;
    }
    web_keyboard_paused_ = true;
    keyboard_velocity_ = {}; sent_ = {}; gate_.reason = web_keyboard_pause_reason_;
  }""",
    )
    text = replace_once(
        text,
        "  void release_keyboard(const std::string& reason) {\n    keyboard_ = navigation_ = false;",
        """  void release_keyboard(const std::string& reason) {
    if (!keyboard_owner_.empty()) {
      last_keyboard_release_reason_ = reason;
      RCLCPP_INFO(get_logger(), "Web base ownership released: %s", reason.c_str());
    }
    web_keyboard_paused_ = false; web_keyboard_pause_reason_.clear(); web_keyboard_sequence_ = 0;
    keyboard_ = navigation_ = false;""",
    )
    text = replace_once(
        text,
        '      if (keyboard_ && detail == keyboard_owner_) release_keyboard("web_keyboard_released");',
        """      if (keyboard_ && (detail == keyboard_owner_ || detail == "__web_operator_release__"))
        release_keyboard("web_keyboard_released");""",
    )
    text = replace_once(
        text,
        "        keyboard_owner_ = detail; keyboard_ = true; navigation_ = false;",
        """        keyboard_owner_ = detail; keyboard_ = true; navigation_ = false;
        web_keyboard_paused_ = false; web_keyboard_pause_reason_.clear(); web_keyboard_sequence_ = 0;""",
    )
    text = replace_once(
        text,
        "          !keyboard_ || navigation_ || !gate_.enabled || !rb::fresh(keyboard_time_, t, 200000000))",
        "          !keyboard_ || navigation_ || !gate_.enabled)",
    )
    text = replace_once(
        text,
        '      const auto v = j.at("velocity").get<std::array<double, 3>>();',
        """      if (!j.contains("sequence") || !j.at("sequence").is_number_unsigned())
        throw std::invalid_argument("Web command sequence required; refresh the console");
      const auto sequence = j.at("sequence").get<uint64_t>();
      if (sequence <= web_keyboard_sequence_)
        throw std::invalid_argument("Stale web command sequence");
      const auto v = j.at("velocity").get<std::array<double, 3>>();""",
    )
    text = replace_once(
        text,
        "      const double scale = std::min(web_keyboard_linear_limit_, gate_.mapping.linear) /",
        """      // Check the old deadline inside the callback as well as the timer.
      // A queued nonzero request must not resume motion after a scheduling gap.
      if (!rb::fresh(keyboard_time_, t, web_keyboard_timeout_ns_))
        pause_web_keyboard("web_command_timeout");
      if (coordinated_mode_ && !luxi::fresh(coordination_.sent_ns, t, 250000000))
        pause_web_keyboard("coordinator_feedback_lost");
      web_keyboard_sequence_ = sequence;
      keyboard_time_ = t;
      if (web_keyboard_paused_) {
        if (v == rb::Velocity{} && rb::fresh(gate_.feedback_time, t, 100000000) &&
            (!coordinated_mode_ || luxi::fresh(coordination_.sent_ns, t, 250000000))) {
          web_keyboard_paused_ = false; web_keyboard_pause_reason_.clear();
        } else {
          success = true; message = "Web motion paused; release WASD to continue"; return true;
        }
      }
      const double scale = std::min(web_keyboard_linear_limit_, gate_.mapping.linear) /""",
    )
    text = replace_once(
        text,
        '          release_keyboard("coordinator_feedback_lost");',
        """          if (!keyboard_owner_.empty()) pause_web_keyboard("coordinator_feedback_lost");
          else release_keyboard("coordinator_feedback_lost");""",
    )
    # Braces prevent the legacy fallback from binding to the outer if.
    text = replace_once(
        text,
        """        if (!luxi::fresh(coordination_.sent_ns, luxi::mono(), 250000000))
          if (!keyboard_owner_.empty()) pause_web_keyboard("coordinator_feedback_lost");
          else release_keyboard("coordinator_feedback_lost");""",
        """        if (!luxi::fresh(coordination_.sent_ns, luxi::mono(), 250000000)) {
          if (!keyboard_owner_.empty()) pause_web_keyboard("coordinator_feedback_lost");
          else release_keyboard("coordinator_feedback_lost");
        }""",
    )
    text = replace_once(
        text,
        '          if (!rb::fresh(keyboard_time_, luxi::mono(), 200000000)) gate_.halt("keyboard_timeout");',
        """          if (!keyboard_owner_.empty()) {
            if (!rb::fresh(keyboard_time_, luxi::mono(), web_keyboard_timeout_ns_))
              pause_web_keyboard("web_command_timeout");
          } else if (!rb::fresh(keyboard_time_, luxi::mono(), 200000000)) gate_.halt("keyboard_timeout");""",
    )
    text = replace_once(
        text,
        '            wanted = keyboard_velocity_;\n            gate_.reason = wanted == rb::Velocity{} ? "keyboard_ready" : "teleop";',
        """            wanted = web_keyboard_paused_ ? rb::Velocity{} : keyboard_velocity_;
            gate_.reason = web_keyboard_paused_ ? web_keyboard_pause_reason_ :
                wanted == rb::Velocity{} ? "keyboard_ready" : "teleop";""",
    )
    text = replace_once(
        text,
        '{"web_keyboard_control", true},',
        """{"web_keyboard_control", true},
                      {"web_keyboard_manual_release", true},
                      {"web_keyboard_paused", web_keyboard_paused_},
                      {"web_keyboard_pause_reason", web_keyboard_pause_reason_},
                      {"web_keyboard_command_timeout_ms", web_keyboard_timeout_ns_ / 1000000},
                      {"web_keyboard_sequence", web_keyboard_sequence_},
                      {"last_keyboard_release_reason", last_keyboard_release_reason_},""",
    )
    return text


def patch_web(text: str) -> str:
    text = replace_once(
        text,
        'let keyboardOwner = "";',
        'let keyboardOwner = "";\nlet keyboardSequence = 0;',
    )
    text = replace_once(
        text,
        "    if (keyboardOwner === owner) { keyboardError = '网页控制已撤销：' + e.message; stopKeyboard(); log(keyboardError); }",
        """    if (keyboardOwner === owner && sequence === keyboardSequence) {
      heldKeys.clear();
      keyboardError = '请求暂时中断，已停车并保留网页控制权；松开 WASD 后再按。' + e.message;
      if (e.status === 409 && e.message.includes('owned by another controller')) {
        keyboardActive = false; keyboardOwner = '';
        keyboardError = '网页控制权已释放，请重新打开开关。';
      }
      buttons(); log(keyboardError);
    }""",
    )
    text = replace_once(
        text,
        "  const owner = keyboardOwner;\n  try {\n    await baseKeyboard('keyboard', JSON.stringify({owner, velocity:",
        "  const owner = keyboardOwner;\n  const sequence = ++keyboardSequence;\n  try {\n    const result = await baseKeyboard('keyboard', JSON.stringify({owner, sequence, velocity:",
    )
    text = replace_once(
        text,
        "Number(heldKeys.has('KeyD')), 0]}));\n  } catch (e)",
        """Number(heldKeys.has('KeyD')), 0]}));
    if (keyboardOwner === owner && sequence === keyboardSequence) {
      if (result.message?.includes('motion paused')) {
        heldKeys.clear(); keyboardError = '指令延迟，已停车并保留网页控制权；松开 WASD 后再按。';
      } else keyboardError = '';
      buttons();
    }
  } catch (e)""",
    )
    text = replace_once(
        text,
        "  keyboardOwner = owner;\n  keyboardStarting = true;",
        "  keyboardOwner = owner; keyboardSequence = 0;\n  keyboardStarting = true;",
    )
    text = replace_once(
        text,
        "    if (document.hidden || !document.hasFocus()) { stopKeyboard(); return; }",
        "    if (document.hidden || !document.hasFocus()) { pauseKeyboard(); return; }",
    )
    text = replace_once(
        text,
        "function stopKeyboard() {",
        """function pauseKeyboard() {
  heldKeys.clear();
  if (keyboardActive) {
    keyboardError = '已停车，网页控制权保留；返回页面后重新按 WASD，关闭开关可释放。';
    buttons();
    const owner = keyboardOwner, sequence = ++keyboardSequence;
    // A zero command may overtake a slow request; its newer sequence prevents
    // the delayed movement request from restarting the chassis afterwards.
    baseKeyboard('keyboard', JSON.stringify({owner, sequence, velocity:[0,0,0]}))
      .catch(e => { if (keyboardOwner === owner) log('停车请求暂未确认，等待指令超时停车：' + e.message); });
  }
}
function stopKeyboard() {""",
    )
    offline_start = text.index("function offline() {")
    offline_end = text.index("\n}", offline_start)
    offline = text[offline_start:offline_end]
    offline = replace_once(offline, "stopKeyboard();", "pauseKeyboard();")
    text = text[:offline_start] + offline + text[offline_end:]
    text = replace_once(
        text,
        "return request('/api/keyboard', {method:'POST', headers:",
        "return request('/api/keyboard', {timeoutMs:command === 'keyboard' ? 800 : 12000, method:'POST', headers:",
    )
    text = replace_once(
        text,
        "  if (e.code === 'Space' || e.code === 'Escape') { e.preventDefault(); stopKeyboard(); return; }",
        "  if (e.code === 'Space' || e.code === 'Escape') { e.preventDefault(); pauseKeyboard(); return; }",
    )
    text = replace_once(
        text,
        "if (['KeyW','KeyA','KeyS','KeyD'].includes(e.code)) { e.preventDefault(); heldKeys.add(e.code);",
        "if (['KeyW','KeyA','KeyS','KeyD'].includes(e.code)) { e.preventDefault(); if (e.repeat) return; heldKeys.add(e.code);",
    )
    text = replace_once(
        text,
        "window.addEventListener('blur', stopKeyboard);",
        "window.addEventListener('blur', pauseKeyboard);",
    )
    text = replace_once(
        text,
        "window.addEventListener('pagehide', stopKeyboard);",
        "window.addEventListener('pagehide', pauseKeyboard);",
    )
    text = replace_once(
        text,
        "document.addEventListener('visibilitychange', () => { if (document.hidden) stopKeyboard(); });",
        "document.addEventListener('visibilitychange', () => { if (document.hidden) pauseKeyboard(); });",
    )
    text = replace_once(
        text,
        "  try { await baseKeyboard('stop', owner); log('底盘已撤销控制许可');",
        "  try { await baseKeyboard('stop', owner || '__web_operator_release__'); log('底盘已释放网页控制权');",
    )
    text = replace_once(
        text,
        "keyboardActive ? '网页已取得底盘控制权，按住 WASD 行驶；关闭开关后停车并让出控制权。'",
        "keyboardActive ? (bs?.web_keyboard_paused ? '指令已暂停，网页权限保留；松开 WASD 后再按，关闭开关可释放。' : '网页已取得底盘控制权，按住 WASD 行驶；关闭开关后停车并让出控制权。')",
    )
    return text


def patch_html(text: str) -> str:
    text = replace_once(
        text,
        "停车并退出键盘控制",
        "停车并释放网页控制权",
    )
    text = replace_once(
        text,
        "使能后直接按住 WASD 行驶，松键停车；空格 / Esc、切换窗口或连接超时会撤销使能。重新进入页面后需再次使能。",
        "按住 WASD 行驶，松键、空格 / Esc 或切换窗口停车并保留网页控制权。指令超过 500 毫秒未更新时停车，松开 WASD 后再按；关闭开关或点击“停车并释放网页控制权”才让出权限。刷新或重新打开页面后，可用释放按钮清理原网页会话。",
    )
    return text


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    patches = {
        "Project/device_adapters/src/r680_adapter.cpp": patch_base,
        "Project/luxi_web_ctrl/web/app.js": patch_web,
        "Project/luxi_web_ctrl/web/index.html": patch_html,
    }
    prepared = []
    for name, patch in patches.items():
        raw = (args.root / name).read_bytes()
        updated = patch(raw.decode().replace("\r\n", "\n"))
        if b"\r\n" in raw:
            updated = updated.replace("\n", "\r\n")
        prepared.append((name, raw, updated.encode()))
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {}
    for name, before, after in prepared:
        for directory, raw in [("before", before), ("candidate", after)]:
            target = args.output / directory / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(raw)
        manifest[name] = {
            "before_sha256": hashlib.sha256(before).hexdigest(),
            "candidate_sha256": hashlib.sha256(after).hexdigest(),
        }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
