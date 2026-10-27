from setuptools import find_packages, setup
setup(name='wheel_gateway', version='0.3.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheel_gateway']),
                  ('share/wheel_gateway', ['package.xml', 'README.md'])],
      install_requires=['setuptools', 'wheel-runtime>=0.3.0'],
      entry_points={'console_scripts':[]})
