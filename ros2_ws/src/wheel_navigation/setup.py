from setuptools import find_packages, setup
setup(name='wheel_navigation', version='0.3.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/wheel_navigation']),
                  ('share/wheel_navigation', ['package.xml', 'README.md', 'SDK_INTEGRATION_README.md'])],
      install_requires=['setuptools', 'wheel-sdk-bridge>=0.3.0', 'PyYAML>=6,<7'],
      entry_points={'console_scripts':[]})
