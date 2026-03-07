#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('offboard_cpp').find('offboard_cpp')
    param_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'ctrl_param.yaml'
    ])

    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    auto_start_animal_testing_arg = DeclareLaunchArgument(
        'auto_start_animal_testing',
        default_value='true',
        description='Automatically start animal testing node'
    )

    offboard_node = Node(
        package='offboard_cpp',
        executable='offboard_node',
        name='offboard_control_node',
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Animal Testing 节点
    animal_testing_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='offboard_cpp',
                executable='animal_testing',
                name='animal_testing_node',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                condition=IfCondition(LaunchConfiguration('auto_start_animal_testing'))
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        auto_start_animal_testing_arg,
        offboard_node,
        animal_testing_node,
    ])
