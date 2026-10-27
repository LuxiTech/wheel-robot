from setuptools import find_packages, setup
setup(name='wheel_planning', version='0.3.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheel_planning']),
                  ('share/wheel_planning', ['package.xml', 'README.md'])],
      install_requires=['setuptools', 'wheel-task-core>=0.3.0', 'wheel-motion-core>=0.3.0'],
      entry_points={'console_scripts':[]})
