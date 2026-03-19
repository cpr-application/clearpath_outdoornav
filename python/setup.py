from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'clearpath_outdoornav_python_api'

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
    maintainer='José Mastrangelo',
    maintainer_email='jmastrangelo@clearpathrobotics.com',
    description='ROS2 package for the Python API of Clearpaths OutdoorNav software',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execute_task_action = endpoints.autonomy.execute_task_action:main',
            'execute_task_srv = endpoints.autonomy.execute_task_srv:main',
            'goto_poi = endpoints.autonomy.goto_poi:main',
            'goto = endpoints.autonomy.goto:main',
            'mission_from_goal = endpoints.autonomy.mission_from_goal:main',
            'mission = endpoints.autonomy.mission:main',
            'pause_autonomy = endpoints.autonomy.pause_autonomy:main',
            'resume_autonomy = endpoints.autonomy.resume_autonomy:main',
            'stop_autonomy = endpoints.autonomy.stop_autonomy:main',
            'add_dock = endpoints.docking.add_dock:main',
            'local_dock = endpoints.docking.local_dock:main',
            'map_dock = endpoints.docking.map_dock:main',
            'survey_dock = endpoints.docking.survey_dock:main',
            'monitor_localization = endpoints.localization.monitor_localization:main',
            'set_datum = endpoints.localization.set_datum:main',
            'delete_log = endpoints.logger.delete_log:main',
            'start_recording = endpoints.logger.start_recording:main',
            'stop_recording = endpoints.logger.stop_recording:main',
            'mission_looped = examples.mission_looped:main',
            'mission_w_monitoring = examples.mission_w_monitoring:main',
        ],
    },
)
