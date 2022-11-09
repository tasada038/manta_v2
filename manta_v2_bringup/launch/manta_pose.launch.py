from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "manta_v2_description"
    xacro_file_name = "manta_v2.xacro"
    rviz_file_name = "urdf_with_pose.rviz"

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

    scan_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='scan',
            output='screen',
            parameters=[{
                'output_frame':'camera_link',
                'approx_sync':True,
            }],
            remappings=[('depth','/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info')],
    )

    base_to_imu = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            # arguments=['0.4', '0.2', '0.0', '3.14/2', '0.0', '0.0', 'base_link', 'imu']
            arguments=['0.4', '0.2', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu']
    )

    base_to_bar30 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'bar30_link']
    )

    pose_to_base = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.22', '0.0', '-0.05', '0.0', '0.0', '0.0', 'pose', 'base_link']
    )

    base_to_camera = Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'camera_link']            
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

    nodes = [
        rviz_node,
        scan_node,
        base_to_imu,
        base_to_camera,
        pose_to_base,
        # base_to_bar30,
        robot_state_pub_node,
        joint_state_pub_gui_node,


    ]

    return LaunchDescription(nodes)
