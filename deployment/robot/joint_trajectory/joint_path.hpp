// Continuous, bounded interpolation of an entire sixteen-axis SDK phase.
// No CAN/ROS access, motor enable, zero offsets, or controller gain changes.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace luxi {
class JointPath {
 public:
  using Axes = std::array<double, 16>;

  JointPath(const std::vector<Axes>& points, const Axes& lower, const Axes& upper,
            const Axes& sdk_speed, double speed, double acceleration = .5)
      : points_(points) {
    if (points.size() < 2 || points.size() > 4096 || !std::isfinite(speed) ||
        speed < .02 || speed > .2 || !std::isfinite(acceleration) ||
        acceleration <= 0 || acceleration > 2.)
      throw std::invalid_argument("Invalid bounded trajectory parameters");
    for (int axis = 0; axis < 16; ++axis) {
      if (!std::isfinite(lower[axis]) || !std::isfinite(upper[axis]) ||
          lower[axis] >= upper[axis] || !std::isfinite(sdk_speed[axis]) || sdk_speed[axis] <= 0)
        throw std::invalid_argument("Invalid SDK trajectory limits");
      limits_[axis] = std::min(speed, sdk_speed[axis]);
    }
    time_.push_back(0);
    for (std::size_t index = 0; index < points_.size(); ++index) {
      double duration = .02;
      for (int axis = 0; axis < 16; ++axis) {
        const double value = points_[index][axis];
        if (!std::isfinite(value) || value < lower[axis] || value > upper[axis])
          throw std::invalid_argument("Trajectory waypoint outside original SDK limits");
        if (index)
          duration = std::max(duration, std::abs(value - points_[index - 1][axis]) / limits_[axis]);
      }
      if (index) time_.push_back(time_.back() + duration);
    }
    std::vector<double> intervals(points_.size() - 1);
    for (std::size_t index = 0; index < intervals.size(); ++index)
      intervals[index] = time_[index + 1] - time_[index];
    // Retiming is local: a tight turn or a nearly stationary IK point must not
    // slow every other part of a long phase. Recompute neighbouring tangents
    // after every duration change, and retain a conservative final bound.
    for (int iteration = 0; iteration < 24; ++iteration) {
      rebuild(intervals);
      bool changed = false;
      for (std::size_t index = 0; index < intervals.size(); ++index) {
        const double factor = required_scale(index, acceleration);
        if (factor > 1.00000001) {
          intervals[index] *= factor * 1.001;
          changed = true;
        }
      }
      if (!changed) break;
    }
    rebuild(intervals);
    double stretch = 1.;
    for (std::size_t index = 0; index < intervals.size(); ++index)
      stretch = std::max(stretch, required_scale(index, acceleration));
    for (double& value : time_) value *= stretch;
    for (Axes& row : tangent_)
      for (double& value : row) value /= stretch;
    if (!std::isfinite(duration()) || duration() > 600)
      throw std::invalid_argument("Trajectory exceeds ten-minute bound");
  }

  double duration() const { return time_.back(); }
  std::size_t size() const { return points_.size(); }
  const std::vector<double>& times() const { return time_; }

  Axes sample(double seconds) const {
    if (!std::isfinite(seconds) || seconds < 0)
      throw std::invalid_argument("Invalid trajectory sample time");
    if (seconds >= duration()) return points_.back();
    const auto found = std::upper_bound(time_.begin(), time_.end(), seconds);
    const std::size_t index = static_cast<std::size_t>(found - time_.begin() - 1);
    const double u = (seconds - time_[index]) / (time_[index + 1] - time_[index]);
    Axes result{};
    for (int axis = 0; axis < 16; ++axis) {
      const auto c = coefficients(index, axis);
      result[axis] = ((c[0] * u + c[1]) * u + c[2]) * u + c[3];
    }
    return result;
  }

 private:
  void rebuild(const std::vector<double>& intervals) {
    time_[0] = 0;
    for (std::size_t index = 0; index < intervals.size(); ++index)
      time_[index + 1] = time_[index] + intervals[index];
    tangent_.assign(points_.size(), Axes{});
    // Shape-preserving Hermite tangents, with zero velocity at the two phase
    // endpoints. Same-direction internal nodes do not introduce artificial stops.
    for (std::size_t index = 1; index + 1 < points_.size(); ++index) {
      const double before = intervals[index - 1], after = intervals[index];
      for (int axis = 0; axis < 16; ++axis) {
        const double left = (points_[index][axis] - points_[index - 1][axis]) / before;
        const double right = (points_[index + 1][axis] - points_[index][axis]) / after;
        if (left * right > 0) {
          const double w1 = 2 * after + before, w2 = after + 2 * before;
          tangent_[index][axis] = (w1 + w2) / (w1 / left + w2 / right);
        }
      }
    }
  }

  double required_scale(std::size_t index, double acceleration) const {
    const double h = time_[index + 1] - time_[index];
    double factor = 1.;
    for (int axis = 0; axis < 16; ++axis) {
      const auto c = coefficients(index, axis);
      double peak = std::max(std::abs(c[2]), std::abs(3 * c[0] + 2 * c[1] + c[2])) / h;
      if (std::abs(c[0]) > 1e-15) {
        const double critical = -c[1] / (3 * c[0]);
        if (critical > 0 && critical < 1)
          peak = std::max(peak, std::abs(3 * c[0] * critical * critical + 2 * c[1] * critical + c[2]) / h);
      }
      const double peak_accel = std::max(std::abs(2 * c[1]), std::abs(6 * c[0] + 2 * c[1])) / (h * h);
      factor = std::max({factor, peak / limits_[axis], std::sqrt(peak_accel / acceleration)});
    }
    return factor;
  }

  std::array<double, 4> coefficients(std::size_t index, int axis) const {
    const double h = time_[index + 1] - time_[index];
    const double start = points_[index][axis], end = points_[index + 1][axis];
    const double m0 = tangent_[index][axis] * h, m1 = tangent_[index + 1][axis] * h;
    return {2 * start - 2 * end + m0 + m1, -3 * start + 3 * end - 2 * m0 - m1, m0, start};
  }

  std::vector<Axes> points_, tangent_;
  std::vector<double> time_;
  Axes limits_{};
};
}  // namespace luxi
