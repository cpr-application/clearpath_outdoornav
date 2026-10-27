from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'clearpath_outdoornav_diagnostics'

setup(
    name=package_name,
    version='2.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='José Mastrangelo',
    maintainer_email='jmastrangelo@clearpathrobotics.com',
    description='Captures a diagnostics snapshot of Clearpath OutdoorNav running on a robot',
    license='Clearpath Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostics_node = clearpath_outdoornav_diagnostics.diagnostics_node:main',
            'nav_diagnostics_capture = '
            'clearpath_outdoornav_diagnostics.nav_diagnostics_capture:main',
        ],
    },
)
