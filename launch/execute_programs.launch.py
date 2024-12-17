from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share_description = FindPackageShare('dh_robot')
    return LaunchDescription([
        DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='dh_robot',
            executable='programme1',
            name='programme1'
        ),
        Node(
            package='dh_robot',
            executable='programme2',
            name='programme2'
        ),
        Node(
            package='dh_robot',
            executable='programme3',
            name='programme3'
        )
    ])