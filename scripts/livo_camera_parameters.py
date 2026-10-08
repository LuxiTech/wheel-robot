"""Camera calibration parameter server required by the ROS 2 vikit adapter."""
import rclpy
from rclpy.node import Node
rclpy.init()
node=Node('parameter_blackboard',automatically_declare_parameters_from_overrides=True)
rclpy.spin(node)
