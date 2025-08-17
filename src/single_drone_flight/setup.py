import os
from glob import glob
from setuptools import setup

package_name = 'single_drone_flight'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],  # No Python packages to install
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Simple ROS2 package to fly a single drone to 5 meters height using PX4 offboard control',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                # Scripts are installed directly via CMakeLists.txt
        ],
    },
)
