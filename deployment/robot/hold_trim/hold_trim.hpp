#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <string>

// Commissioning-only bounded, single-joint static error correction.
// No CAN, ROS, enable/disable, threading, or autonomous restart here.
class HoldTrim {
 public:
  using Q = std::array<double, 16>;
  Q output{}, goal{}, initial{}, reference{}, lower{}, upper{};
  std::string phase = "idle", reason = "not_requested";
  int axis = -1;
  double elapsed = 0, settled = 0;
  static constexpr double rate = .002;  // 0.115 degrees/s output slew
  static constexpr double max_change = .08, tolerance = .003;
  bool active() const { return phase == "correcting"; }
  void freeze(const std::string& why) {
    if (active()) { phase = "stopped"; reason = why; }
  }
  bool start(int selected, double target, const Q& sent, const Q& measured,
             const Q& lo, const Q& hi, std::string& error) {
    if (active()) { error = "hold trim already active"; return false; }
    if (selected < 0 || selected >= 16 || selected % 8 == 7 || !std::isfinite(target)) {
      error = "one arm joint required; grippers excluded"; return false;
    }
    for (int i = 0; i < 16; ++i) {
      if (!std::isfinite(sent[i]) || !std::isfinite(measured[i]) ||
          !std::isfinite(lo[i]) || !std::isfinite(hi[i]) || lo[i] >= hi[i] ||
          sent[i] < lo[i] || sent[i] > hi[i] || measured[i] < lo[i] || measured[i] > hi[i]) {
        error = "invalid or out-of-range initial state"; return false;
      }
    }
    if (target < lo[selected] || target > hi[selected] ||
        std::abs(target-measured[selected]) > max_change ||
        std::abs(sent[selected]-measured[selected]) > .12) {
      error = "requested correction exceeds commissioning bounds"; return false;
    }
    axis = selected; output = initial = sent; reference = measured; goal = measured;
    goal[axis] = target; lower = lo; upper = hi;
    elapsed = settled = 0; phase = "correcting"; reason = "bounded_static_error_correction";
    return true; // First output remains EXACTLY the previous support setpoint.
  }
  void step(const Q& measured, const Q& velocity, double dt, bool healthy) {
    if (!active()) return;
    if (!healthy || !std::isfinite(dt) || dt <= 0 || dt > .05) {
      freeze("feedback_or_loop_invalid"); return;
    }
    for (int i = 0; i < 16; ++i) {
      if (!std::isfinite(measured[i]) || !std::isfinite(velocity[i]) ||
          measured[i] < lower[i] || measured[i] > upper[i] || std::abs(velocity[i]) > .08) {
        freeze("measured_motion_or_limits"); return;
      }
      if (i != axis && std::abs(measured[i]-reference[i]) > .01) {
        freeze("unselected_joint_moved"); return;
      }
    }
    elapsed += dt;
    if (elapsed > 60) { freeze("correction_timeout"); return; }
    if (std::abs(output[axis]-measured[axis]) > .12) { freeze("following_error"); return; }
    const double error = goal[axis]-measured[axis];
    if (std::abs(error) <= tolerance) {
      settled += dt;
      if (settled >= .5) { phase = "ready"; reason = "measured_static_arrival"; }
      return;
    }
    settled = 0;
    const double delta = std::clamp(.3*error, -rate, rate)*dt;
    const double value = output[axis]+delta;
    if (value < lower[axis] || value > upper[axis] || std::abs(value-initial[axis]) > max_change) {
      freeze("correction_saturated"); return; // No integrator windup.
    }
    output[axis] = value;
  }
};
