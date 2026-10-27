from setuptools import find_packages, setup
setup(name='wheel_execution', version='0.3.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheel_execution']),
                  ('share/wheel_execution', ['package.xml', 'README.md'])],
      install_requires=['setuptools', 'wheel-sdk-bridge>=0.3.0', 'wheel-planning>=0.3.0'],
      entry_points={'console_scripts':[]})
