from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'clearpath_outdoornav_simulation'

setup(
    name=package_name,
    version='2.0.0',
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
    maintainer='José Mastrangelo',
    maintainer_email='jmastrangelo@clearpathrobotics.com',
    description='Clearpath OutdoorNav simulation package',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'localization_sim = clearpath_outdoornav_simulation.localization_sim:main',
            'platform_sim = clearpath_outdoornav_simulation.platform_sim:main',
        ],
    },
)
