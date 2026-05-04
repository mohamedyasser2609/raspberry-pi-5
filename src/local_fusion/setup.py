from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'local_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #  IMPORTANT: add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='raspberrypi',
    maintainer_email='raspberrypi@todo.todo',
    description='Local Fusion Package',

    license='TODO',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [],
    },
)
