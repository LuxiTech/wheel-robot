from setuptools import setup
setup(name='wheel_grasp_core', version='0.1.0', packages=['wheel_grasp'],
      data_files=[('share/ament_index/resource_index/packages',['resource/wheel_grasp_core']),
                  ('share/wheel_grasp_core',['package.xml']),
                  ('share/wheel_grasp_core/launch',['launch/description.launch.py'])],
      install_requires=['numpy','scipy','opencv-python-headless','PyYAML'],
      entry_points={'console_scripts':['wheel-grasp=wheel_grasp.cli:main',
                                      'wheel-grasp-rgbd=wheel_grasp.ros:main',
                                      'wheel-grasp-state=wheel_grasp.ros_state:main']})
