#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <set>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/srv/get_motion_plan.hpp>
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("wheel_retime",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_model_loader::RobotModelLoader loader(node,"robot_description");
  auto model=loader.getModel();
  auto service=node->create_service<moveit_msgs::srv::GetMotionPlan>("/wheel/retime",
    [model](const std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request> request,
            std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Response> response) {
      auto& out=response->motion_plan_response;auto& input=request->motion_plan_request;
      out.error_code.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      if(input.group_name!="both_arms" || input.trajectory_constraints.constraints.size()<2) return;
      moveit::core::RobotState state(model);state.setToDefaultValues();
      moveit::core::robotStateMsgToRobotState(input.start_state,state);
      robot_trajectory::RobotTrajectory trajectory(model,input.group_name);
      for(const auto& waypoint: input.trajectory_constraints.constraints) {
        if(waypoint.joint_constraints.size()!=14) return;
        std::set<std::string> names;
        for(const auto& joint:waypoint.joint_constraints) {
          if(!model->getJointModelGroup("both_arms")->hasJointModel(joint.joint_name) || !names.insert(joint.joint_name).second || !std::isfinite(joint.position)) return;
          state.setVariablePosition(joint.joint_name,joint.position);
        }
        state.update();if(!state.satisfiesBounds()) return;
        trajectory.addSuffixWayPoint(state,0.01);
      }
      // Zero path tolerance: preserve the sampled dual-arm Cartesian path.
      trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.0,0.01,0.001);
      if(!timing.computeTimeStamps(trajectory,input.max_velocity_scaling_factor,input.max_acceleration_scaling_factor)) return;
      trajectory.getRobotTrajectoryMsg(out.trajectory);
      out.trajectory_start=input.start_state;out.group_name=input.group_name;
      out.error_code.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    });
  rclcpp::spin(node);rclcpp::shutdown();
}
