import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'joy_motor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='kdrturan',
    maintainer_email='aturan446@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_motor = joy_motor_pkg.JoyMotorNode:main',
            'joy_to_wrench = joy_motor_pkg.joy_to_wrench_node:main',
            'minirov_joy = joy_motor_pkg.minirov_joy_node:main',
            'thruster_allocator = joy_motor_pkg.thruster_allocator_node:main',
        ],
    },
)
