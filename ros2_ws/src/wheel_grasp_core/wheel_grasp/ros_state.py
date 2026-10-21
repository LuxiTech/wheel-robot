"""Robot-local SDK feedback -> named measured joints. No motion publisher."""
import json
import time
from .sdk import arm_feedback,LiftMapping


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from luxi_interfaces.msg import ArmState
    class StateBridge(Node):
        def __init__(self):
            super().__init__('wheel_sdk_measured_joints')
            offset=self.declare_parameter('lift_offset_m',float('nan')).value
            direction=self.declare_parameter('lift_direction',1).value
            self.mapping=LiftMapping(direction,offset,
                self.declare_parameter('device_min_um',-180000).value,
                self.declare_parameter('device_max_um',180000).value)
            self.publisher=self.create_publisher(JointState,'joint_states',10)
            self.arms=self.create_subscription(ArmState,'/luxi/arms/state',self.on_arms,5)
            self.lift=self.create_subscription(String,'/luxi/lift/state',self.on_lift,5)
        def on_arms(self,msg):
            try:
                age=time.monotonic_ns()-msg.receive_mono_ns
                if not 0<=age<=150_000_000: raise ValueError('Stale/local-clock mismatch in arms')
                values=arm_feedback(dict(position=list(msg.position),valid=list(msg.valid),age_ns=list(msg.age_ns)))
                out=JointState();out.header=msg.header;out.name=list(values);out.position=list(values.values())
                self.publisher.publish(out)
            except (ValueError,KeyError) as exc:self.get_logger().warning(str(exc))
        def on_lift(self,msg):
            try:
                data=json.loads(msg.data)
                age=time.monotonic_ns()-int(data['receive_mono_ns'])
                if (data.get('valid') is not True or not 0<=age<=150_000_000
                        or not 0<=float(data['effective_age_ms'])<=150):
                    raise ValueError('Invalid/stale lift feedback')
                q=self.mapping.to_joint(data['current_um'])
                ns=int(data['ros_time_ns'])
                out=JointState();out.header.stamp.sec=ns//1_000_000_000;out.header.stamp.nanosec=ns%1_000_000_000
                out.name=['lift_joint'];out.position=[q];self.publisher.publish(out)
            except (ValueError,KeyError) as exc:self.get_logger().warning(str(exc))
    rclpy.init();node=StateBridge()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
