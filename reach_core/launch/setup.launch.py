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
from launch.conditions import IfCondition


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "visualize_results",
            description="Package to look for study parameters yaml file.",
            default_value="true"
        )
    )

    visualize_results = LaunchConfiguration("visualize_results")

    load_point_cloud_server_node = Node(
        package="reach_core",
        executable="load_point_cloud_server_node",
        name="load_point_cloud_server_node",
        output="screen",
        parameters=[]
    )

    nodes_to_run = [load_point_cloud_server_node,
                    ]

    return LaunchDescription(declared_arguments + nodes_to_run)