#include "joint_path.hpp"

#include <cassert>
#include <iostream>
#include <limits>

int main() {
  using Axes = luxi::JointPath::Axes;
  Axes low{}, high{}, speed{}, start{}, middle{}, finish{};
  low.fill(-2); high.fill(2); speed.fill(1);
  middle[0] = .3; middle[8] = -.2;
  finish[0] = .8; finish[8] = -.7;
  luxi::JointPath path({start, middle, finish}, low, high, speed, .2, .5);
  assert(path.sample(0) == start);
  assert(path.sample(path.duration()) == finish);
  assert(path.sample(path.duration() + 1) == finish);
  const double epsilon = 1e-5;
  const double join = path.times()[1];
  const auto left = path.sample(join - epsilon), center = path.sample(join), right = path.sample(join + epsilon);
  assert(center == middle);
  assert((right[0] - left[0]) / (2 * epsilon) > .01);
  assert(std::abs((center[0] - left[0]) / epsilon - (right[0] - center[0]) / epsilon) < 1e-4);
  const int samples = 20000;
  const double dt = path.duration() / samples;
  auto previous = path.sample(0);
  Axes previous_velocity{};
  for (int index = 1; index <= samples; ++index) {
    const auto value = path.sample(index * dt);
    for (int axis = 0; axis < 16; ++axis) {
      const double velocity = (value[axis] - previous[axis]) / dt;
      assert(std::abs(velocity) < .20001);
      if (index > 1) assert(std::abs(velocity - previous_velocity[axis]) / dt < .5001);
      assert(value[axis] >= std::min(start[axis], finish[axis]) - 1e-10);
      assert(value[axis] <= std::max(start[axis], finish[axis]) + 1e-10);
      previous_velocity[axis] = velocity;
    }
    previous = value;
  }
  assert(std::abs(path.sample(epsilon)[0] / epsilon) < 1e-4);
  assert(std::abs((finish[0] - path.sample(path.duration() - epsilon)[0]) / epsilon) < 1e-4);
  Axes reversal = finish;
  reversal[0] = -.3;
  luxi::JointPath turn({start, middle, reversal}, low, high, speed, .2);
  for (int index = 0; index <= samples; ++index) {
    const auto value = turn.sample(turn.duration() * index / samples);
    assert(value[0] >= -.300001 && value[0] <= .300001);
  }
  auto rejects = [&](const std::vector<Axes>& points, double velocity) {
    bool rejected = false;
    try { luxi::JointPath invalid(points, low, high, speed, velocity); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  };
  rejects({start}, .2);
  rejects({start, finish}, .2001);
  rejects({start, finish}, 0);
  Axes nan = finish;
  nan[3] = std::numeric_limits<double>::quiet_NaN();
  rejects({start, nan}, .2);
  finish[3] = 2.001;
  rejects({start, finish}, .2);
  std::cout << "Continuous waypoint passage, bounds, velocity/acceleration, reversal and invalid inputs passed\n";
}
