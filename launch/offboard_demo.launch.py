#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


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
    
    auto_start_demo_arg = DeclareLaunchArgument(
        'auto_start_demo',
        default_value='false',
        description='Automatically start demo node'
    )
    
    # 主控制节点 - 运行状态机和控制逻辑
    offboard_node = Node(
        package='offboard_cpp',
        executable='offboard_node',
        name='offboard_control_node',
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # 如果需要重映射话题，在这里添加
        ]
    )
    
    # Demo节点 - 发布轨迹目标（可选）
    # 延迟5秒启动，确保主节点先准备好
    offboard_demo = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='offboard_cpp',
                executable='offboard_demo',
                name='offboard_demo_node',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                condition=IfCondition(LaunchConfiguration('auto_start_demo'))
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        auto_start_demo_arg,
        offboard_node,
        offboard_demo,
    ])
