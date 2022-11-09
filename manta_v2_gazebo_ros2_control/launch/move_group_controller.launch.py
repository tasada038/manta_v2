from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os

def generate_launch_description():
    # Constants for paths to different files and folders
    gazebo_pkg_name = 'manta_v2_gazebo_ros2_control'
    robot_name_in_model = 'manta_v2'
    urdf_file_path = 'urdf/manta_v2_moveit.urdf'
    #world_file_path = 'worlds/myrobot.world'
    world_file_path = 'worlds/yolo_test.world'
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'


    gazebo_pkg_share = FindPackageShare(package=gazebo_pkg_name).find(gazebo_pkg_name)
    world_path = os.path.join(gazebo_pkg_share, world_file_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world_path}.items(),
    )

    urdf_model_path = os.path.join(gazebo_pkg_share, urdf_file_path)

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    gazebo_robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    ) 

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name_in_model,
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val,
                                   ],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller'],
    #     output='screen'
    # )

    load_rtop_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'rtop_controller'],
        output='screen'
    )
    load_rmid_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'rmid_controller'],
        output='screen'
    )
    load_rbtm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'rbtm_controller'],
        output='screen'
    )
    load_ltop_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'ltop_controller'],
        output='screen'
    )
    load_lmid_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'lmid_controller'],
        output='screen'
    )
    load_lbtm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'lbtm_controller'],
        output='screen'
    )

    load_lbtm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'manta_controller'],
        output='screen'
    )

    nodes = [
        # Gazebo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[
                    load_rtop_controller,
                    load_rmid_controller,
                    load_rbtm_controller,
                    load_ltop_controller,
                    load_lmid_controller,
                    load_lbtm_controller,
                    # load_joint_trajectory_controller,
                ],
            )
        ),
        gazebo,
        gazebo_robot_state_pub_node,
        spawn_entity,
    ]

    return LaunchDescription(nodes)
