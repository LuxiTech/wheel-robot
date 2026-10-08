"""Publish simulation raw measurements to FAST-LIVO2 and return its odometry.

Run inside the ROS 2 container. No simulator poses are published as odometry.
"""
import json
import os
import subprocess
import signal
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,Image
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg,CustomPoint

FOLDER=Path('/workspace/outputs/fast_livo')
DIAGNOSTIC_NO_VISION=os.environ.get('LIVO_DIAGNOSTIC_NO_VISION')=='1'


def stamp(value):
    from builtin_interfaces.msg import Time
    ns=int(round(float(value)*1e9))
    return Time(sec=ns//1000000000,nanosec=ns%1000000000)


class Bridge(Node):
    def __init__(self,session):
        super().__init__('wheel_robot_livo_bridge')
        self.session=session;self.seq=0;self.odom_count=0
        self.imu=self.create_publisher(Imu,'/sim/imu',1000)
        self.lidar=self.create_publisher(CustomMsg,'/sim/lidar',100)
        self.image=self.create_publisher(Image,'/sim/image',100)
        self.create_subscription(Odometry,'/aft_mapped_to_init',self.odometry,100)
        self.create_timer(.01,self.publish_frame)

    def odometry(self,msg):
        self.odom_count+=1
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        result=dict(run_id=self.session['run_id'],stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],
                    source='FAST-LIVO2',pipeline='LIO_only_diagnostic' if DIAGNOSTIC_NO_VISION else 'LIVO',samples=self.odom_count)
        tmp=FOLDER/'odometry.tmp'
        tmp.write_text(json.dumps(result));os.replace(tmp,FOLDER/'odometry.json')
        if self.odom_count%10==0:
            print('LIVO_ODOMETRY',json.dumps(result),flush=True)

    def publish_frame(self):
        if not self.imu.get_subscription_count() or not self.lidar.get_subscription_count() or (not DIAGNOSTIC_NO_VISION and not self.image.get_subscription_count()):
            return
        path=FOLDER/self.session['run_id']/f'{self.seq:08d}.npz'
        if not path.exists():
            return
        with np.load(path) as packet:
            for sample in packet['imu']:
                msg=Imu();msg.header.stamp=stamp(sample[0]);msg.header.frame_id='imu'
                msg.orientation_covariance[0]=-1.
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,sample[1:4])
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,sample[4:7])
                self.imu.publish(msg)
            t=float(packet['stamp'])
            cloud=CustomMsg();cloud.header.stamp=stamp(t);cloud.header.frame_id='lidar'
            cloud.timebase=int(round(t*1e9));cloud.point_num=len(packet['points'])
            for i,xyz in enumerate(packet['points']):
                pt=CustomPoint();pt.x,pt.y,pt.z=map(float,xyz)
                pt.line=i%16;pt.tag=16;pt.reflectivity=100
                # This is an instantaneous simulated scan, not a fabricated
                # rotating scan with incorrect per-point acquisition times.
                pt.offset_time=0
                cloud.points.append(pt)
            self.lidar.publish(cloud)
            rgb=np.asarray(packet['rgb'],dtype=np.uint8)
            image=Image();image.header.stamp=stamp(t);image.header.frame_id='camera_optical'
            image.height,image.width=rgb.shape[:2];image.encoding='rgb8';image.step=image.width*3
            image.data=rgb.tobytes();self.image.publish(image)
        path.unlink()
        self.seq+=1


def main():
    FOLDER.mkdir(exist_ok=True)
    after=float(os.environ.get('LIVO_SESSION_AFTER','0'))
    while not (FOLDER/'session.json').exists() or (FOLDER/'session.json').stat().st_mtime<after:
        time.sleep(.1)
    rclpy.init()
    while rclpy.ok():
        session=json.loads((FOLDER/'session.json').read_text())
        children=[];node=None
        try:
            children.append(subprocess.Popen(['python3','/workspace/scripts/livo_camera_parameters.py','--ros-args','--params-file','/workspace/config/fast_livo_camera.yaml'],start_new_session=True))
            mapper=['ros2','run','fast_livo','fastlivo_mapping','--ros-args','--params-file','/workspace/config/fast_livo_sim.yaml']
            if DIAGNOSTIC_NO_VISION:mapper+=['-p','common.img_en:=0']
            children.append(subprocess.Popen(mapper,start_new_session=True))
            node=Bridge(session)
            while rclpy.ok():
                if any(p.poll() is not None for p in children):raise RuntimeError('FAST-LIVO2 process exited')
                if json.loads((FOLDER/'session.json').read_text())['run_id']!=session['run_id']:
                    break
                rclpy.spin_once(node,timeout_sec=.05)
        finally:
            if node is not None:node.destroy_node()
            for child in children:
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=5.)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL)


if __name__=='__main__':main()
