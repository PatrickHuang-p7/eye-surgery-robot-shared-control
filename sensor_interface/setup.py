import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pq',
    maintainer_email='pq@todo.todo',
    description='ROS 2 sensor interface package for various sensors.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_interface = sensor_interface.sensor_node:main',
            'hardware_adc = sensor_interface.hardware_adc_node:main',
        ],
    },
)
