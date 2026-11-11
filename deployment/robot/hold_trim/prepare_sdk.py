#!/usr/bin/env python3
"""Generate an isolated SDK candidate; never overwrite live source or restart anything."""
import argparse
import hashlib
from pathlib import Path
import shutil

EXPECTED = 'fb8443cdb96a067689f2357fe28a087f9b20c8d387ca3b8ea6dc157984885b41'
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('source', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
raw = a.source.read_bytes()
if hashlib.sha256(raw).hexdigest() != EXPECTED:
    raise SystemExit('SDK source changed; re-audit before preparing this patch')
s = raw.decode().replace('\r\n', '\n')
def replace(old, new):
    global s
    if s.count(old) != 1:
        raise SystemExit('Expected exactly one anchor: '+old[:70])
    s = s.replace(old, new)
replace('#include "openarm_monitor.hpp"', '#include "openarm_monitor.hpp"\n#include "hold_trim.hpp"')
replace('  luxi::Json json_;', '''  HoldTrim hold_trim_;
  bool allow_hold_trim_ = false;
  std::string hold_trim_id_, hold_trim_fingerprint_;
  int64_t hold_trim_tick_ = 0;
  luxi::Json json_;''')
replace('  void leave_manual(const std::string& reason) {', '''  void leave_manual(const std::string& reason) {
    hold_trim_.freeze("manual_owner_left");''')
replace('  void latch_manual_fault(const std::string& reason) {', '''  void latch_manual_fault(const std::string& reason) {
    hold_trim_.freeze(reason);''')
replace('    monitor_only_ = declare_parameter("monitor_only", false);', '''    monitor_only_ = declare_parameter("monitor_only", false);
    allow_hold_trim_ = declare_parameter("allow_hold_trim", false);''')
anchor='  bool startup_command('
start=s.index(anchor)
body=s.index('{',start)+1
s=s[:body]+'''
    if (command == "hold_trim" || command == "hold_trim_stop")
      return hold_trim_command(command, detail, message);
    if (hold_trim_.active()) {
      if (command == "joint_stop" || command == "stop" || command == "disable")
        hold_trim_.freeze("operator_stop");
      else { message = "Stop hold_trim before changing control modes"; return false; }
    }
''' +s[body:]
replace('  bool startup_command(', '''  bool hold_trim_command(const std::string& command, const std::string& detail, std::string& message) {
    if (!allow_hold_trim_) { message = "hold_trim is not commissioned"; return false; }
    int axis = -1; double target = 0; std::string id, fingerprint;
    try {
      auto j = luxi::Json::parse(detail);
      if (!j.is_object() || !j.contains("request_id") || !j["request_id"].is_string())
        throw std::invalid_argument("request_id required");
      id = j["request_id"].get<std::string>();
      if (id.empty() || id.size() > 96) throw std::invalid_argument("invalid request_id");
      if (command == "hold_trim") {
        if (!j.contains("axis") || !j["axis"].is_number_integer() ||
            !j.contains("target_rad") || !j["target_rad"].is_number())
          throw std::invalid_argument("integer axis and finite target_rad required");
        axis = j["axis"].get<int>(); target = j["target_rad"].get<double>();
        fingerprint = j.dump();
      }
    } catch (const std::exception& e) { message = e.what(); return false; }
    if (command == "hold_trim_stop") {
      if (id != hold_trim_id_) { message = "hold_trim request mismatch"; return false; }
      hold_trim_.freeze("operator_stop"); message = "Holding last support output"; return true;
    }
    if (id == hold_trim_id_) {
      message = "hold_trim request already recorded";
      return fingerprint == hold_trim_fingerprint_;
    }
    if (!enabled_ || startup_owned_ || manual_phase_ != "holding" ||
        manual_fault_latched_ || joint_active() || !manual_feedback_rejection(luxi::mono()).empty()) {
      message = "Requires enabled stationary manual hold with fresh feedback"; return false;
    }
    if (!hold_trim_.start(axis, target, sent_, state_.position, cfg_.lo, cfg_.hi, message)) return false;
    hold_trim_id_ = id; hold_trim_fingerprint_ = fingerprint;
    hold_trim_tick_ = luxi::mono();
    manual_target_ = sent_; // Preserve all 16 supporting outputs on acceptance.
    message = "Bounded single-joint hold correction accepted; no enable transition";
    return true;
  }
  bool startup_command(''')
replace('''      command_.position = manual_target_;
      command_.phase = "manual_pose_hold";''','''      if (hold_trim_.active()) {
        const double trim_dt = (t - hold_trim_tick_) * 1e-9;
        hold_trim_tick_ = t;
        hold_trim_.step(state_.position, state_.velocity, trim_dt, manual_feedback_fresh(t));
        manual_target_ = hold_trim_.output;
      }
      command_.position = manual_target_;
      command_.phase = hold_trim_.active() ? "manual_pose_hold_trim" : "manual_pose_hold";''')
replace('''          try {
            if (joint_active() && s->data == "joint_stop:" + manual_generation_) {''', '''          try {
            // Any lifecycle intervention ends trimming; the existing lifecycle rules still apply.
            hold_trim_.freeze("driver_lifecycle_intervention");
            if (joint_active() && s->data == "joint_stop:" + manual_generation_) {''')
replace('''        {"control_generation", control_generation_}, {"support_target", sent_},''','''        {"control_generation", control_generation_}, {"support_target", sent_},
        {"hold_trim", {{"available", allow_hold_trim_}, {"request_id", hold_trim_id_},
            {"phase", hold_trim_.phase}, {"reason", hold_trim_.reason}, {"axis", hold_trim_.axis},
            {"goal", hold_trim_.goal}, {"output", hold_trim_.output},
            {"output_rate_rad_s", HoldTrim::rate}, {"max_output_change_rad", HoldTrim::max_change}}},''')
a.output.mkdir(parents=True, exist_ok=False)
(a.output/'openarm_adapter.cpp').write_text(s)
shutil.copy2(Path(__file__).with_name('hold_trim.hpp'), a.output/'hold_trim.hpp')
shutil.copy2(a.source.with_name('openarm_monitor.hpp'), a.output/'openarm_monitor.hpp') if a.source.with_name('openarm_monitor.hpp').exists() else None
print(a.output.resolve())
