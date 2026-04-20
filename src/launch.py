from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([     
        Node(
            package='puzzlebot_evasion_final',
            executable='bug3',
            name='bug3',
            output='screen'
        ),
        Node(
            package='puzzlebot_evasion_final',
            executable='odom',
            name='odom',
            output='screen'
        ),
    ])

