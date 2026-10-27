from setuptools import find_packages, setup
setup(name='wheel_runtime', version='0.3.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheel_runtime']),
                  ('share/wheel_runtime', ['package.xml', 'README.md'])],
      install_requires=['setuptools', 'wheel-task-core>=0.3.0', 'wheel-perception-core>=0.3.0', 'wheel-motion-core>=0.3.0', 'wheel-navigation>=0.3.0', 'wheel-perception>=0.3.0', 'wheel-execution>=0.3.0', 'wheel-planning>=0.3.0', 'PyYAML>=6,<7'],
      entry_points={'console_scripts':['wheel-robot=wheel_runtime.main:main', 'wheel-grasp=wheel_runtime.cli:main']})
