from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    pkg_share_description = FindPackageShare('dh_robot')
    default_rviz_config_path = PathJoinSubstitution([pkg_share_description, 'rviz', 'dh_robot.rviz'])
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
        ),
        DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ'
        ),
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
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])