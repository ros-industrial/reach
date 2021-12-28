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
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            description="Package where to find rviz file under /rviz subfolder.",
            default_value="reach_core"
        )
    )

    visualize_results = LaunchConfiguration("visualize_results")
    rviz_config_package = LaunchConfiguration("rviz_config_package")

    # rviz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(rviz_config_package),
         "rviz",
         "reach_study_config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(visualize_results),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        # arguments=["-d", rviz_config_file],
        parameters=[],
    )

    load_point_cloud_server_node = Node(
        package="reach_core",
        executable="load_point_cloud_server_node",
        name="load_point_cloud_server_node",
        output="screen",
        parameters=[]
    )

    nodes_to_run = [load_point_cloud_server_node,
                    rviz_node]

    return LaunchDescription(declared_arguments + nodes_to_run)