from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uart_comstack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('uart_comstack/launch/*.py')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],

    install_requires=[
        'setuptools',
        'pyserial',
        'pynmea2',
        'transforms3d'
    ],

    zip_safe=True,

    maintainer='raspberrypi',
    maintainer_email='raspberrypi@todo.todo',
    description='UART communication and GPS Odometry package',
    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'tm4c_bridge = uart_comstack.tm4c_bridge:main',
            'uart_comstack_node = uart_comstack.uart_comstack_node:main',
            'odometry_node = uart_comstack.odometry_node:main',
            'gps_node = uart_comstack.gps_node:main',
        ],
    },
)
