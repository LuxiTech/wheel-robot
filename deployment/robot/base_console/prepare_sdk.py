#!/usr/bin/env python3
"""Prepare a reviewable SDK candidate from live source without installing it."""

import argparse
import hashlib
import json
from pathlib import Path


def replace_once(text: str, before: str, after: str) -> str:
    if text.count(before) != 1:
        raise ValueError(f"Expected one SDK anchor: {before[:100]!r}")
    return text.replace(before, after, 1)


def patch_base(text: str) -> str:
    text = replace_once(
        text, "#include <thread>", "#include <thread>\n#include <deque>\n#include <set>"
    )
    text = replace_once(
        text,
        "  bool keyboard_ = false;",
        """  bool keyboard_ = false;
  std::string keyboard_owner_;
  std::set<std::string> keyboard_owners_;
  std::deque<std::string> keyboard_owner_order_;
  static constexpr double web_keyboard_linear_limit_ = .05;
  bool keyboard_coordinator_idle(int64_t t) const {
    return !coordinated_mode_ ||
        (luxi::fresh(coordination_.sent_ns, t, 250000000) &&
         (coordination_.phase == "disabled" || coordination_.phase == "halt"));
  }
  void release_keyboard(const std::string& reason) {
    keyboard_ = navigation_ = false;
    keyboard_owner_.clear(); keyboard_velocity_ = {}; keyboard_time_ = 0;
    sent_ = {};
    gate_.halt(reason);
  }
  bool web_keyboard_command(const std::string& command, const std::string& detail,
                            bool& success, std::string& message) {
    const bool scoped_stop = command == "stop" && !detail.empty();
    const bool start = command == "keyboard_start";
    const bool update = command == "keyboard";
    if (!scoped_stop && !start && !update) return false;
    if (!coordinated_mode_ && keyboard_owner_.empty() && detail.empty()) return false;
    // Preserve legacy array commands only for independent, unowned sessions.
    if (update && !coordinated_mode_ && keyboard_owner_.empty() &&
        !detail.empty() && detail.front() == '[') return false;
    success = false;
    if (scoped_stop) {
      if (keyboard_ && detail == keyboard_owner_) release_keyboard("web_keyboard_released");
      success = true; message = "Web control released; stale owners cannot stop a new controller";
      return true;
    }
    try {
      const auto t = luxi::mono();
      if (start) {
        if (detail.empty() || detail.size() > 96) throw std::invalid_argument("Web control owner required");
        if (keyboard_ && gate_.enabled && detail == keyboard_owner_) {
          success = true; message = "Web control already accepted"; return true;
        }
        if (keyboard_owners_.count(detail)) throw std::invalid_argument("Expired web control; turn the switch on again");
        if (receive_only_ || gate_.enabled || !keyboard_coordinator_idle(t) || gate_.menu || gate_.stop ||
            stationary_samples_ < 2 || !rb::fresh(gate_.feedback_time, t, 100000000))
          throw std::invalid_argument("Base busy or not ready; stop Quest control before enabling web control");
        keyboard_owners_.insert(detail); keyboard_owner_order_.push_back(detail);
        if (keyboard_owner_order_.size() > 256) {
          keyboard_owners_.erase(keyboard_owner_order_.front()); keyboard_owner_order_.pop_front();
        }
        keyboard_owner_ = detail; keyboard_ = true; navigation_ = false;
        keyboard_velocity_ = {}; keyboard_time_ = t; sent_ = {};
        gate_.enabled = true; gate_.reason = "web_keyboard_ready";
        success = true; message = "Web base control acquired"; return true;
      }
      const auto j = luxi::Json::parse(detail);
      if (!j.is_object() || !j.contains("owner") || !j.at("owner").is_string() ||
          j.at("owner").get<std::string>() != keyboard_owner_ || keyboard_owner_.empty() ||
          !keyboard_ || navigation_ || !gate_.enabled || !rb::fresh(keyboard_time_, t, 200000000))
        throw std::invalid_argument("Web control expired or owned by another controller");
      const auto v = j.at("velocity").get<std::array<double, 3>>();
      for (double value : v)
        if (!std::isfinite(value) || std::abs(value) > 1) throw std::invalid_argument("Invalid web velocity");
      const double scale = std::min(web_keyboard_linear_limit_, gate_.mapping.linear) /
          std::max(1., std::hypot(v[0], v[1]));
      keyboard_velocity_ = {v[0] * scale, v[1] * scale, v[2] * .30};
      keyboard_time_ = t; success = true; message = "Web command accepted";
    } catch (const std::exception& error) {
      // A rejected or late request cannot release a different owner's hold.
      message = error.what();
    }
    return true;
  }""",
    )
    text = replace_once(
        text,
        "    return !receive_only_ && stationary_samples_ >= 2 && candidate.start(t);",
        "    return !keyboard_ && !receive_only_ && stationary_samples_ >= 2 && candidate.start(t);",
    )
    text = replace_once(
        text,
        "if (input_fresh && input_ordered) gate_.note_input_arrival(q->receive_mono_ns);",
        "if (input_fresh && input_ordered && !keyboard_) gate_.note_input_arrival(q->receive_mono_ns);",
    )
    text = replace_once(
        text,
        "} else if (y_toggle && !coordinated_mode_) {",
        "} else if (y_toggle && !coordinated_mode_ && !keyboard_) {",
    )
    text = replace_once(
        text,
        "    const bool changed = coordination_.accept(data, t);\n    if (!coordination_.live(t))",
        """    const bool changed = coordination_.accept(data, t);
    if (keyboard_ && gate_.enabled) {
      // Incoming Quest packets and rejected prepare attempts never steal the
      // web lease. Explicit shared parking remains a stop action.
      if (coordination_.phase == "stop" || coordination_.phase == "park" ||
          (changed && coordination_.phase == "halt"))
        release_keyboard("shared_operator_stop");
      if (coordination_.phase == "prepare" || coordination_.phase == "active")
        coordination_.blocked = true;
      return;
    }
    if (!coordination_.live(t))""",
    )
    text = replace_once(
        text,
        "      if (coordinated_mode_) {\n        if (!coordination_.live(luxi::mono()))",
        """      if (coordinated_mode_ && keyboard_) {
        if (!luxi::fresh(coordination_.sent_ns, luxi::mono(), 250000000))
          release_keyboard("coordinator_feedback_lost");
      } else if (coordinated_mode_) {
        if (!coordination_.live(luxi::mono()))""",
    )
    text = replace_once(
        text,
        '        if (coordinated_mode_ && coordination_.phase == "active" && !gate_.enabled)',
        '        if (keyboard_ && !gate_.enabled) release_keyboard(gate_.reason);\n        if (coordinated_mode_ && coordination_.phase == "active" && !gate_.enabled)',
    )
    text = replace_once(
        text,
        '{"manual_twist", true},',
        """{"manual_twist", true},
                      {"web_keyboard_control", true},
                      {"web_keyboard_linear_limit", std::min(web_keyboard_linear_limit_, gate_.mapping.linear)},
                      {"keyboard_owner", keyboard_owner_},
                      {"keyboard_takeover_available", keyboard_coordinator_idle(luxi::mono()) && !gate_.enabled},""",
    )
    text = replace_once(
        text,
        '          if (coordinated_mode_) {\n            if (req->command == "stop"',
        '''          if (web_keyboard_command(req->command, req->detail, res->success, res->message)) return;
          if (coordinated_mode_) {
            if (req->command == "stop"''',
    )
    return text


def patch_manager(text: str) -> str:
    text = replace_once(
        text,
        "  void hold_all(const std::string& why) {",
        """  void hold_all(const std::string& why) {
    if (base_status_.is_object() && base_status_.value("enabled", false) && base_status_.value("input_source", "") == "keyboard")
      epoch_ = owner_ + ":" + std::to_string(++epoch_count_);""",
    )
    function = text.index("  bool shared_command(const std::string& s) {")
    start = text.index('    if (s == "start") {', function)
    insertion = start + len('    if (s == "start") {')
    return (
        text[:insertion]
        + """
      if (base_status_.is_object() && base_status_.value("enabled", false) && base_status_.value("input_source", "") == "keyboard") {
        reason_ = "web_base_control_active"; return false;
      }
"""
        + text[insertion:]
    )


def patch_web(text: str) -> str:
    text = replace_once(
        text, 'let keyboardError = "";', 'let keyboardError = "";\nlet keyboardOwner = "";'
    )
    text = replace_once(
        text,
        "        else allow &&= ['disabled','hold'].includes(s.control?.phase);",
        "        else allow &&= ['disabled','hold'].includes(s.control?.phase);\n        allow &&= !(s.base_status?.enabled && s.base_status.input_source === 'keyboard');",
    )
    text = replace_once(
        text,
        "  else if (bs.shared_enable) reason = '统一 X 模式使用 Quest 摇杆；独立键盘使能已关闭。';",
        """  else if (bs.shared_enable && !bs.web_keyboard_control) reason = '底盘驱动尚未支持网页接管。';
  else if (bs.shared_enable && !keyboardActive && !bs.keyboard_takeover_available && !bs.enabled) reason = 'Quest 遥操已占用，请先停止遥操，再打开网页控制。';""",
    )
    text = replace_once(
        text,
        "  else if (bs.enabled && !keyboardActive) reason = '已有控制许可，请先停车退出，再使能本页键盘控制。';",
        """  else if (!keyboardActive && bs.stationary === false) reason = '等待底盘反馈确认停稳后再开启网页控制。';
  else if (bs.enabled && (!keyboardActive || (bs.keyboard_owner && bs.keyboard_owner !== keyboardOwner))) reason = '底盘正由其他控制端使用，请先停车并释放控制权。';""",
    )
    text = text.replace(
        "'键盘控制：正在开启…' : keyboardActive ? '键盘控制：开启' : '键盘控制：关闭'",
        "'网页控制：正在开启…' : keyboardActive ? '网页控制：开启' : '网页控制：关闭'",
    )
    text = replace_once(
        text,
        "keyboardActive ? '键盘控制已启用，按住 WASD 行驶；松键停车。' : reason || '准备就绪，打开键盘控制开关即可，无需令牌。'",
        "keyboardActive ? '网页已取得底盘控制权，按住 WASD 行驶；关闭开关后停车并让出控制权。' : reason || '准备就绪，无需连接 Quest；打开网页控制开关即可使用。'",
    )
    text = replace_once(
        text,
        "  const active = keyboardActive;\n  keyboardActive = false; heldKeys.clear(); buttons();\n  if (active) baseKeyboard('stop').catch(e => log('停车请求失败：' + e.message));",
        """  const owner = keyboardOwner;
  keyboardActive = false; keyboardOwner = ''; heldKeys.clear(); buttons();
  if (owner) baseKeyboard('stop', owner).catch(e => log('停车请求失败：' + e.message));""",
    )
    text = replace_once(
        text,
        "  keyboardPending = true;\n  try {\n    await baseKeyboard('keyboard', JSON.stringify([Number(heldKeys.has('KeyW'))-Number(heldKeys.has('KeyS')), Number(heldKeys.has('KeyA'))-Number(heldKeys.has('KeyD'))]));\n  } catch (e) { keyboardError = '键盘控制已撤销：' + e.message; stopKeyboard(); log(keyboardError); }",
        """  keyboardPending = true;
  const owner = keyboardOwner;
  try {
    await baseKeyboard('keyboard', JSON.stringify({owner, velocity:[Number(heldKeys.has('KeyW'))-Number(heldKeys.has('KeyS')), Number(heldKeys.has('KeyA'))-Number(heldKeys.has('KeyD')), 0]}));
  } catch (e) {
    if (keyboardOwner === owner) { keyboardError = '网页控制已撤销：' + e.message; stopKeyboard(); log(keyboardError); }
  }""",
    )
    text = replace_once(
        text,
        "  keyboardStarting = true; keyboardError = ''; buttons();\n  try {\n    await baseKeyboard('keyboard_start');\n    keyboardActive = true; heldKeys.clear();",
        """  const owner = 'web-' + Date.now().toString(36) + '-' + Math.random().toString(36).slice(2);
  keyboardOwner = owner;
  keyboardStarting = true; keyboardError = ''; buttons();
  try {
    await baseKeyboard('keyboard_start', owner);
    if (keyboardOwner !== owner) { await baseKeyboard('stop', owner); return; }
    keyboardActive = true; heldKeys.clear();""",
    )
    text = replace_once(
        text,
        "  } catch(e) { keyboardError = '键盘启用失败：' + e.message; log(keyboardError); }",
        "  } catch(e) { keyboardError = '网页控制启用失败：' + e.message; stopKeyboard(); log(keyboardError); }",
    )
    text = replace_once(
        text,
        "  keyboardActive = false; heldKeys.clear(); keyboardError = ''; buttons();\n  try { await baseKeyboard('stop');",
        "  const owner = keyboardOwner;\n  keyboardActive = false; keyboardOwner = ''; heldKeys.clear(); keyboardError = ''; buttons();\n  try { await baseKeyboard('stop', owner);",
    )
    return text


def patch_html(text: str) -> str:
    text = replace_once(
        text,
        '<h2 id="keyboard-title">键盘测试控制</h2>',
        '<h2 id="keyboard-title">网页底盘控制</h2>',
    )
    text = replace_once(
        text,
        "打开键盘控制开关即可使用，无需 Quest。测试速度上限 0.05 m/s。",
        "没有遥操占用时，打开开关即可使用 WASD，无需连接 Quest。关闭开关后停车并让出控制权；网页与 Quest 互斥控制。测试速度上限 0.05 m/s。",
    )
    text = replace_once(
        text, "disabled>键盘控制：关闭</button>", "disabled>网页控制：关闭</button>"
    )
    return text


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    patches = {
        "Project/device_adapters/src/r680_adapter.cpp": patch_base,
        "Project/control_manager/src/control_manager.cpp": patch_manager,
        "Project/luxi_web_ctrl/web/app.js": patch_web,
        "Project/luxi_web_ctrl/web/index.html": patch_html,
    }
    prepared = []
    for name, patch in patches.items():
        raw = (args.root / name).read_bytes()
        decoded = raw.decode().replace("\r\n", "\n")
        updated = patch(decoded)
        if b"\r\n" in raw:
            updated = updated.replace("\n", "\r\n")
        prepared.append((name, raw, updated.encode()))
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {}
    for name, before, after in prepared:
        for directory, data in [("before", before), ("candidate", after)]:
            path = args.output / directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        manifest[name] = {
            "before_sha256": hashlib.sha256(before).hexdigest(),
            "candidate_sha256": hashlib.sha256(after).hexdigest(),
        }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
