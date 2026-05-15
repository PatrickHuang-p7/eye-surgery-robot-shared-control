import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch 文件
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py*'))),
        
        # Config 文件 (YAML)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        
        # ⬇️ ***CRITICAL FIX: 添加这一行来安装 URDF 文件夹*** ⬇️
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pq',
    maintainer_email='pq@todo.todo',
    description='ROS 2 driver and control package for UART serial servo arm.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'arm_driver = arm_driver.arm_driver_node:main',
            'arm_control = arm_driver.arm_control_node:main',
            'pygame_haptic_sim = arm_driver.pygame_haptic_sim_node:main',
            'pygame_keyboard_preview = arm_driver.pygame_keyboard_preview:main',
            'demo_actions = arm_driver.demo_actions:main',
            'kinematics_visualization = arm_driver.kinematics_visualization:main',
            'shared_control_param_sweep = arm_driver.shared_control_param_sweep:main',
            'shared_control_experiment_analyzer = arm_driver.shared_control_experiment_analyzer:main',
            'shared_control_experiment_plotter = arm_driver.shared_control_experiment_plotter:main',
            'shared_control_batch_bag_recorder = arm_driver.shared_control_batch_bag_recorder:main',
            'shared_control_trial_logger = arm_driver.shared_control_trial_logger:main',
        ],
        'hardware_interface': [
            'UartServoHardwareInterface = arm_driver.servo_hardware_interface:UartServoHardwareInterface',
        ]
    },
)
