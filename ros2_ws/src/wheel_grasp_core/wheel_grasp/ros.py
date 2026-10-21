"""Optional ROS 2 atomic subscription. Records observations; never moves robot."""
import json
from pathlib import Path
import time
import cv2
import numpy as np
from .rgbd import Observation,validate,save_observation


def from_ros(message):
    c,d,i=message.color,message.depth,message.camera_info
    stamp=lambda x:(x.stamp.sec,x.stamp.nanosec,x.frame_id)
    if not all(stamp(x.header)==stamp(message.header) for x in (c,d,i)):
        raise ValueError('ROS RGBD headers mismatch')
    if (c.camera_id!=message.camera_id or d.camera_id!=message.camera_id
            or c.sequence!=message.sequence or d.sequence!=message.sequence
            or c.receive_mono_ns!=d.receive_mono_ns
            or c.width!=d.width or c.height!=d.height or i.width!=d.width or i.height!=d.height):
        raise ValueError('ROS RGBD group mismatch')
    if len(d.data)!=d.width*d.height*4: raise ValueError('Malformed ROS depth payload')
    bgr=cv2.imdecode(np.frombuffer(bytes(c.data),np.uint8),cv2.IMREAD_COLOR)
    if bgr is None: raise ValueError('Invalid ROS JPEG')
    meta=dict(schema='luxi_rgbd_v1',camera_id=message.camera_id,pair_id=message.pair_id,
        sequence=str(message.sequence),aligned_to_color=True,frame_id=message.header.frame_id,
        width=d.width,height=d.height,stamp=dict(sec=message.header.stamp.sec,nanosec=message.header.stamp.nanosec),
        receive_mono_ns=str(d.receive_mono_ns),age_ms=0,mock=d.timestamp_quality=='mock_host',
        pairing='sdk_frameset',hardware_synchronized=False,calibration_available=bool(i.distortion_model),
        ros_projection_compatible=i.distortion_model=='plumb_bob',
        intrinsics=dict(K=list(i.k),D=list(i.d),R=list(i.r),P=list(i.p),distortion_model=i.distortion_model),
        color=dict(encoding=c.encoding,bytes=len(c.data),device_timestamp_ns=str(c.device_timestamp),timestamp_quality=c.timestamp_quality),
        depth=dict(encoding=d.encoding,unit='metre',byte_order='little-endian',step=d.width*4,bytes=len(d.data),
            device_timestamp_ns=str(d.device_timestamp),timestamp_quality=d.timestamp_quality,
            value_semantics='source_depth_z_resampled_to_color'))
    rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
    depth=np.frombuffer(bytes(d.data),'<f4').reshape(d.height,d.width).copy()
    validate(meta,rgb,depth)
    meta['source_age_unverified']=True # Cross-machine ROS stamp is not assumed synchronized.
    return Observation(meta,rgb,depth,time.monotonic())


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from luxi_interfaces.msg import RGBDFrame
    class Receiver(Node):
        def __init__(self):
            super().__init__('wheel_grasp_rgbd_receiver')
            camera=self.declare_parameter('camera','head_rgb').value
            self.folder=Path(self.declare_parameter('output','outputs/rgbd').value)
            self.folder.mkdir(parents=True,exist_ok=True);self.last=None;self.count=0
            self.sub=self.create_subscription(RGBDFrame,'/luxi/cameras/'+camera+'/rgbd',self.receive,qos_profile_sensor_data)
        def receive(self,msg):
            try:
                obs=from_ros(msg)
                if obs.key==self.last: return
                self.count+=1
                save_observation(self.folder/(str(time.time_ns())+'.npz'),obs)
                self.last=obs.key
            except (ValueError,KeyError,OSError) as exc: self.get_logger().error(str(exc))
    rclpy.init();node=Receiver()
    try: rclpy.spin(node)
    finally: node.destroy_node();rclpy.shutdown()
