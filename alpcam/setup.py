import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'alpcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brian',
    maintainer_email='berickson@gmail.com',
    description='Alp stereo camera driver for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = alpcam.camera_node:main',
            'find_camera = alpcam.device:main',
        ],
    },
)
