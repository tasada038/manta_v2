from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ballast_top_node = Node(
        package='ballast_control_node',
        executable='joy_to_topballast',
        output="screen",
    )

    ballast_bottom_node = Node(
        package='ballast_control_node',
        executable='joy_to_bottomballast',
        output="screen",
    )

    light_node = Node(
        package='lumenlight_topic',
        executable='joy_to_light',
        output="screen",
    )

    bar30_node = Node(
        package='ms5837_node',
        executable='bar30_node',
        respawn="true",
    )
    
    bno055_node = Node(
        package='ros2_bno055',
        executable='bno055',
        respawn="true",
    )

    return LaunchDescription([
        ballast_top_node,
        ballast_bottom_node,
        light_node,
        bar30_node,
        bno055_node,
    ])
