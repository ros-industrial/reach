from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    # Command,
    # FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameters_package",
            description="Package to look for study parameters yaml file.",
            default_value="reach_demo"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parameters_filename",
            description="YAML file for study parameters.",
            default_value="params.yaml"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz",
                              default_value="true",
                              description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("xacro_file",
                              default_value="reach_study.xacro",
                              description="Xacro file to parse.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("moveit_config_file",
                              default_value="reach_study.srdf.xacro",
                              description="Moveit config xacro file to parse.")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

    parameters_package = LaunchConfiguration("parameters_package")
    parameters_filename = LaunchConfiguration("parameters_filename")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    controllers_file = LaunchConfiguration("controllers_file")

    study_parameters = PathJoinSubstitution(
        [FindPackageShare(parameters_package), "config", parameters_filename]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("reach_demo"), "model", LaunchConfiguration("xacro_file")]
            ),
        ]
    )
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("reach_demo"), "model", moveit_config_file]
            ),
        ]
    )

    controllers = PathJoinSubstitution(
        [FindPackageShare(parameters_package), "model/motoman_sia20d/config", controllers_file]
    )

    kinematics_yaml = load_yaml("reach_demo", "model/motoman_sia20d/config/kinematics.yaml")
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    robot_reach_study_node = Node(
        package="reach_core",
        executable="robot_reach_study_node",
        name="robot_reach_study_node",
        output="screen",
        parameters=[
            study_parameters,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        # arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            # ompl_planning_pipeline_config,
            robot_description_kinematics,
            # robot_description_planning,
        ],
    )

    nodes_to_run = [robot_reach_study_node,
                    control_node,
                    robot_state_publisher_node,
                    joint_state_broadcaster_spawner,
                    rviz_node]

    return LaunchDescription(declared_arguments + nodes_to_run)