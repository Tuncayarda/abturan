import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'stm32_bridge'

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
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='zeybekyasin13@gmail.com',
    description='UART bridge to the STM32 actuator board (6 ESC, stepper, LED)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stm32_bridge = stm32_bridge.stm32_bridge_node:main',
        ],
    },
)
