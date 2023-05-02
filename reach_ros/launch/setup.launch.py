import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    package_dir = get_package_share_directory('reach_ros')
    rviz_config = os.path.join(
        package_dir, 'launch', 'reach_study_config.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    robot_description_file_path = LaunchConfiguration(
        'robot_description_file_path')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description_file_path',
        description='Path to the robot_description URDF file')

    robot_description = ParameterValue(Command(['xacro ',
                                                robot_description_file_path]),
                                       value_type=str)

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description}])

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_robot_description_cmd)

    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld
