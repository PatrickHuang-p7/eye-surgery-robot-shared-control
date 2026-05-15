from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    # 获取包路径
    pkg_path = FindPackageShare('arm_driver')
    
    # 1. 启动 Gazebo Sim (Ignition Gazebo)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'  # 使用空世界，或指定你的世界文件
        }.items()
    )
    
    # 2. 生成你的机器人模型到Gazebo中
    # 假设你有一个URDF/xacro文件
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_path, 'urdf', 'arm.xacro']),  # 修改为你的机器人文件
        ' use_sim_time:=true'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )
    
    # 3. 在Gazebo中生成机器人实体
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # 4. （可选）启动 ROS-Gazebo 桥接，用于特定话题/服务
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
    #     ],
    #     output='screen'
    # )
    
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        # bridge,
    ])