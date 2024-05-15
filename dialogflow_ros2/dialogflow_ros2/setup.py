from setuptools import setup
import os

package_name = 'dialogflow_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/dialogflow.launch.py']),
        (os.path.join('share', package_name, 'resource'), ['resource/dialogflow_ros2']),
        (os.path.join('share', package_name, 'dialogflow_ros2/utils'), ['dialogflow_ros2/utils/converters.py']),
        (os.path.join('share', package_name, 'dialogflow_ros2/utils'), ['dialogflow_ros2/utils/output.py']),
        (os.path.join('share', package_name, 'config'), ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Carlos Manzanares Serrano',
    maintainer_email='jc.manzanares.serrano@gmail.com',
    description='Dialogflow client ros2',
    license='Apache License, Version 2.0',
    tests_require=[''],
    entry_points={
        'console_scripts': [
            'dialogflow_client = dialogflow_ros2.dialogflow_client:main'
        ],
    },
)