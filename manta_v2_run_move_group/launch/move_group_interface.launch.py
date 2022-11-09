import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    description_folder = "manta_v2_description"
    moveit_config_folder = "manta_v2_moveit_config"
    xacro_file = "urdf/manta_v2.xacro"
    srdf_file = "config/manta_v2.srdf"
    rviz_file = "rviz/run_move_group.rviz"

    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory(description_folder),
            xacro_file,
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        moveit_config_folder, srdf_file
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        moveit_config_folder, "config/kinematics.yaml"
    )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="manta_v2_run_move_group",
        package="manta_v2_run_move_group",
        executable="manta_v2_run_move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml],
    )

    return LaunchDescription([move_group_demo])
