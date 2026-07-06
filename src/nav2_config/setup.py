import os
from glob import glob # استيراد الـ glob عشان نقرأ الفولدرات
from setuptools import find_packages, setup

package_name = 'nav2_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. تعديل: عشان ينقل كل ملفات الـ launch اللي جوه الفولدر مش ملف واحد بس
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        
        # 2. إضافة: عشان ينقل فولدر الـ config كله بملفات الـ yaml اللي جواه
        ('share/' + package_name + '/config', glob('config/*')),
        
        # 3. إضافة: عشان ينقل فولدر الـ urdf كله بملفات الـ xacro والـ urdf اللي جواه
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspberrypi',
    maintainer_email='raspberrypi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ws_nav2_bridge = nav2_config.websocket_to_nav2:main',
        ],
    },
)
