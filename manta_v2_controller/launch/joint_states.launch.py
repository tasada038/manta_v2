from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    wing_node = Node(
        package='manta_v2_controller',
        executable='wing_motion_node',
        output="screen",
    )

    walking_node = Node(
        package='manta_v2_controller',
        executable='walking_motion_node',
        output="screen",
    )


    return LaunchDescription([
        wing_node,
        walking_node,
    ])