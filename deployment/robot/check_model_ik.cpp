// Read-only SDK model verification. Constructs no ROS node or hardware adapter.
#include <luxi/ik.hpp>

#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  try {
    luxi::ArmConfig config(luxi::config(argv[2]));
    luxi::Ik ik(argv[1], config);
    std::ifstream input(argv[3]);
    luxi::Json samples;
    input >> samples;
    luxi::Json result = { {"commands_sent", false}, {"samples", luxi::Json::array()} };
    for (const auto& sample : samples) {
      const auto q = sample.get<std::array<double, 16>>();
      ik.sync(q);
      luxi::Json entry = luxi::Json::array();
      for (int side = 0; side < 2; ++side) {
        const auto pose = ik.fk(side);
        const auto rotation = luxi::rot(pose).toRotationMatrix();
        const auto position = luxi::pos(pose);
        luxi::Json matrix = luxi::Json::array();
        for (int row = 0; row < 4; ++row) {
          luxi::Json values = luxi::Json::array();
          for (int col = 0; col < 4; ++col)
            values.push_back(row == 3 ? (col == 3 ? 1.0 : 0.0)
                                     : (col == 3 ? position[row] : rotation(row, col)));
          matrix.push_back(values);
        }
        entry.push_back(matrix);
      }
      result["samples"].push_back(entry);
    }
    auto q = config.home;
    ik.sync(q);
    luxi::Targets target;
    target.right = ik.fk(0);
    target.left = ik.fk(1);
    target.right.position.z += 0.01;
    target.left.position.z += 0.01;
    target.grippers = {q[7], q[15]};
    for (int iteration = 0; iteration < 160; ++iteration) {
      const auto next = ik.solve(target, q);
      for (int index = 0; index < 16; ++index) {
        if (!std::isfinite(next[index]) ||
            std::abs(next[index] - q[index]) > config.speed[index] * .033 + 1e-8)
          throw std::runtime_error("Invalid IK output or velocity limit violation");
      }
      q = next;
    }
    ik.sync(q);
    const double right_error = (luxi::pos(ik.fk(0)) - luxi::pos(target.right)).norm();
    const double left_error = (luxi::pos(ik.fk(1)) - luxi::pos(target.left)).norm();
    if (right_error > .02 || left_error > .02)
      throw std::runtime_error("IK convergence check failed");
    result["ik_error_m"] = {right_error, left_error};
    std::cout << result.dump(2) << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
