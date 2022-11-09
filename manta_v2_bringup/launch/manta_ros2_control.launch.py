from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "manta_v2_description"
    gz_pacakges_name = "manta_v2_gazebo_ros2_control"
    xacro_file_name = "manta_v2.xacro"
    rviz_file_name = "urdf_with_imu.rviz"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(packages_name), "urdf", xacro_file_name]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "rviz", rviz_file_name]
    )

    base_to_imu = Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            # arguments=['0.4', '0.2', '0.0', '3.14/2', '0.0', '0.0', 'base_link', 'imu']
            arguments=['0.4', '0.2', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu']
    )

    base_to_bar30 = Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'bar30_link']
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # ros2_control
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(gz_pacakges_name),
            "config",
            "manta_v2_trajectory_controller.yaml",
        ]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    joint_state_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_controller", "-c", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    nodes = [
        rviz_node,
        base_to_imu,
        # base_to_bar30,
        # robot_state_pub_node,
        #joint_state_pub_gui_node,
        ros2_control_node,
        joint_state_controller_spawner,
        joint_trajectory_controller_spawner,
    ]

    return LaunchDescription(nodes)
