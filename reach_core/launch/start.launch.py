from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    # Command,
    # FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


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

    parameters_package = LaunchConfiguration("parameters_package")
    parameters_filename = LaunchConfiguration("parameters_filename")

    study_parameters = PathJoinSubstitution(
        [FindPackageShare(parameters_package), "config", parameters_filename]
    )

    robot_reach_study_node = Node(
        package="reach_core",
        executable="robot_reach_study_node",
        name="robot_reach_study_node",
        output="screen",
        parameters=[study_parameters]
    )

    nodes_to_run = [robot_reach_study_node]

    return LaunchDescription(declared_arguments + nodes_to_run)