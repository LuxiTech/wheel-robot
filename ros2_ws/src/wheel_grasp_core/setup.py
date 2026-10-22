from setuptools import find_packages, setup
setup(name='wheel_grasp_core', version='0.2.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/wheel_grasp_core']),
                  ('share/wheel_grasp_core',['package.xml']),
                  ('share/wheel_grasp_core/launch',['launch/description.launch.py'])],
      install_requires=['numpy','scipy','opencv-python-headless','PyYAML'],
      entry_points={'console_scripts':['wheel-robot=wheel_grasp.main:main',
                                      'wheel-grasp=wheel_grasp.cli:main',
                                      'wheel-grasp-rgbd=wheel_grasp.vision.ros:main',
                                      'wheel-grasp-state=wheel_grasp.motion.ros_state:main']})
