#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = FindPackageShare('offboard_cpp').find('offboard_cpp')
    
    # 参数文件路径
    param_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'ctrl_param.yaml'
    ])
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # 蜂群节点启动，有几台就几个 node
    drone1_node = Node(
        package="offboard_cpp",
        executable="offboard_node",
        name='drone1_node',
        namespace="drone1",
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )

    drone2_node = Node(
        package="offboard_cpp",
        executable="offboard_node",
        name='drone2_node',
        namespace="drone2",
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )

    drone3_node = Node(
        package="offboard_cpp",
        executable="offboard_node",
        name='drone3_node',
        namespace="drone3",
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        drone1_node,
        drone2_node,
        drone3_node,
    ])
