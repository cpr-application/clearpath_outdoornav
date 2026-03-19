from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'clearpath_outdoornav_api_lib'

setup(
    name=package_name,
    version='2.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmastrangelo',
    maintainer_email='jmastrangelo@clearpathrobotics.com',
    description='A library of ROS2 endpoint wrappers to ease OutdoorNav APi development',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
