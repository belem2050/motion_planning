import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_directory = get_package_share_directory('robotic_setup')
    urdf_file = os.path.join(package_share_directory, 'urdf', 'setup.urdf')
    srdf_file = os.path.join(package_share_directory, 'urdf', 'setup.srdf')
    rviz_config_file = os.path.join(package_share_directory, 'rviz', 'display.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='robotic_setup',
            executable='motion_planner',
            name='motion_planner',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_file).read()},
                {'robot_description_semantic': open(srdf_file).read()}
            ]
        ),
    ])
