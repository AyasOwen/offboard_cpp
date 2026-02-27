#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    启动 Offboard 控制节点
    
    使用方法:
        ros2 launch offboard_cpp offboard_control.launch.py
    
    参数:
        use_sim_time: 是否使用仿真时间 (默认: false)
    """
    
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
        default_value='true',
        description='Use simulation time'
    )
    
    # 主控制节点
    offboard_node = Node(
        package='offboard_cpp',
        executable='offboard_node',
        name='offboard_control_node',
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        offboard_node,
    ])
