from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_arm_driver = get_package_share_directory('arm_driver')
    
    # 1. 定义串口参数 (默认 /dev/serial0)
    arm_port_arg = DeclareLaunchArgument(
        'arm_port',
        default_value='/dev/serial0', 
        description='Serial port for the servo mechanical arm.'
    )
    
    # 2. 处理 URDF 模型 (用于 Rviz 显示)
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_arm_driver, 'urdf', 'arm.xacro']),
        ' serial_port:=', LaunchConfiguration('arm_port')
    ])
    
    robot_description_param = {'robot_description': robot_description}

    # 3. 节点: 机器人状态发布器 (RSP)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param],
        output='screen'
    )

    # 4. 节点: 你的 Python 驱动 (Driver)
    arm_driver_node = Node(
        package='arm_driver',
        executable='arm_driver',
        name='arm_driver_node',
        parameters=[
            {'serial_port': LaunchConfiguration('arm_port')}
        ],
        output='screen',
        emulate_tty=True
    )

    # 5. 节点: Rviz2 (可视化)
    # 我们先尝试加载一个默认配置，如果你保存过 config，可以在这里指定路径
    rviz_config_file = PathJoinSubstitution([pkg_arm_driver, 'rviz', 'arm.rviz'])
    
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # 如果你还没有保存过 .rviz 文件，这一行可能会报个小警告，但 Rviz 依然会启动
        # arguments=['-d', rviz_config_file] 
    )

    # 6. 动作: 自动归位 (Auto-Home)
    # 在驱动启动 5 秒后，发送一条指令让所有舵机回到 0.0 位置
    # 初始位置设为全 0 (或者你可以修改 data 里的数值)
    move_to_home = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', 
            '/arm/joint_command', 
            'std_msgs/msg/Float32MultiArray', 
            '{data: [0.0, 0.0, 0.0, 0.0, 0.0]}'
        ],
        output='screen'
    )
    
    # 使用 TimerAction 延迟执行，确保驱动节点已经准备好接收指令
    delayed_home_action = TimerAction(
        period=5.0, # 延迟 5 秒
        actions=[move_to_home]
    )


    # 假设你已经定义了 sensor_port_arg (虽然I2C不需要，但保持参数兼容性)
    sensor_interface_node = Node(
        package='sensor_interface',
        executable='sensor_interface',
        name='sensor_interface_node',
        output='screen'
    # ⚠️ 注意：I2C 传感器不需要 serial_port 参数，但需要 I2C 权限
    )



    return LaunchDescription([
        arm_port_arg,
        robot_state_publisher_node,
        arm_driver_node,
        node_rviz,           # 启动 Rviz
        delayed_home_action,
        sensor_interface_node  
    ])