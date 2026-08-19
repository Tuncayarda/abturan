import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rpi_cam_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='zeybekyasin13@gmail.com',
    description='Camera control node for the Raspberry Pi UDP/RTP video bridge',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rpi_cam_node = rpi_cam_bridge.rpi_cam_node:main',
        ],
    },
)
