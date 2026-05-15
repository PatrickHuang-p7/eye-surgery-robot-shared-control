import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_share_dir = get_package_share_directory('arm_driver')
    sensor_share_dir = get_package_share_directory('sensor_interface')
    default_arm_control_params = os.path.join(
        arm_share_dir, 'config', 'arm_control_shared_balanced.yaml'
    )
    default_pygame_params = os.path.join(
        arm_share_dir, 'config', 'pygame_haptic_sim_shared_debug.yaml'
    )
    default_hardware_adc_params = os.path.join(
        sensor_share_dir, 'config', 'hardware_adc_shared_debug.yaml'
    )

    # 1. 机械臂串口保持为 launch 参数，其他节点用各自的 ROS 参数默认值启动
    arm_port_arg = DeclareLaunchArgument(
        'arm_port',
        default_value='/dev/serial0',
        description='Serial port for the servo mechanical arm.'
    )
    hardware_adc_params_arg = DeclareLaunchArgument(
        'hardware_adc_params',
        default_value=default_hardware_adc_params,
        description='Params YAML for hardware_adc_node.',
    )
    pygame_params_arg = DeclareLaunchArgument(
        'pygame_params',
        default_value=default_pygame_params,
        description='Params YAML for pygame_haptic_sim_node.',
    )
    pygame_sdl_video_driver_arg = DeclareLaunchArgument(
        'pygame_sdl_video_driver',
        default_value='dummy',
        description='SDL_VIDEODRIVER for pygame_haptic_sim_node. Use dummy on headless SSH sessions.',
    )
    arm_control_params_arg = DeclareLaunchArgument(
        'arm_control_params',
        default_value=default_arm_control_params,
        description='Params YAML for arm_control_node.',
    )

    arm_serial_port = LaunchConfiguration('arm_port')
    hardware_adc_params = LaunchConfiguration('hardware_adc_params')
    pygame_params = LaunchConfiguration('pygame_params')
    pygame_sdl_video_driver = LaunchConfiguration('pygame_sdl_video_driver')
    arm_control_params = LaunchConfiguration('arm_control_params')

    # 2. 机械臂驱动节点
    arm_driver_node = Node(
        package='arm_driver',
        executable='arm_driver',
        name='arm_driver_node',
        output='screen',
        parameters=[
            {'serial_port': arm_serial_port},
        ],
        emulate_tty=True
    )

    # 3. ADS1115 硬件采集节点
    hardware_adc_node = Node(
        package='sensor_interface',
        executable='hardware_adc',
        name='hardware_adc_node',
        output='screen',
        parameters=[hardware_adc_params],
        emulate_tty=True
    )

    # 4. Pygame 虚拟夹具节点
    pygame_haptic_sim_node = Node(
        package='arm_driver',
        executable='pygame_haptic_sim',
        name='pygame_haptic_sim_node',
        output='screen',
        parameters=[pygame_params],
        additional_env={
            'SDL_VIDEODRIVER': pygame_sdl_video_driver,
            'PYGAME_HIDE_SUPPORT_PROMPT': '1',
        },
        emulate_tty=True
    )

    # 5. 高级控制节点
    arm_control_node = Node(
        package='arm_driver',
        executable='arm_control',
        name='arm_control_node',
        output='screen',
        parameters=[arm_control_params],
        emulate_tty=True
    )

    return LaunchDescription([
        arm_port_arg,
        hardware_adc_params_arg,
        pygame_params_arg,
        pygame_sdl_video_driver_arg,
        arm_control_params_arg,
        arm_driver_node,
        hardware_adc_node,
        pygame_haptic_sim_node,
        arm_control_node,
    ])
