from setuptools import find_packages, setup
setup(name='wheel_sdk_bridge', version='0.3.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheel_sdk_bridge']),
                  ('share/wheel_sdk_bridge', ['package.xml', 'README.md'])],
      install_requires=['setuptools', 'wheel-perception-core>=0.3.0', 'numpy>=1.24,<3'],
      entry_points={'console_scripts':['wheel-grasp-state=wheel_sdk_bridge.ros_state:main', 'wheel-grasp-rgbd=wheel_sdk_bridge.rgbd_ros:main']})
