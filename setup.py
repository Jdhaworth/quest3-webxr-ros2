from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'quest3_webxr_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'webxr_app'), glob('webxr_app/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Oculus Quest 3 WebXR integration with ROS2 using TF2 and topics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'quest3_webxr_server = quest3_webxr_ros2.webxr_server:main',
        ],
    },
)
