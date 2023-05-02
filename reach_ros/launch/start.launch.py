import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description_file_path',
            description='Path to the robot_description URDF file'),
        DeclareLaunchArgument(
            'robot_description_semantic_config_path',
            description='The semantic description that corresponds to the URDF'),
        DeclareLaunchArgument(
            'robot_description_kinematics_path',
            description='Load default settings for kinematics; these settings are overridden by settings in a node\'s namespace'),
        DeclareLaunchArgument(
            'robot_description_joints_limits_path',
            description='Load updated joint limits (override information from URDF)'),
        DeclareLaunchArgument(
            'config_file_path',
            description='YAML configuration file for the reach study'),
        DeclareLaunchArgument(
            'config_name',
            description='Arbitrary configuration name for the reach study', default_value="reach_study"),
        DeclareLaunchArgument(
            'results_dir_path',
            description='Location in which reach study results will be saved', default_value="/tmp"),
        OpaqueFunction(function=launch)])


def launch(context, *args, **kwargs):
    robot_description_file_path = LaunchConfiguration(
        'robot_description_file_path')
    robot_description_semantic_config_path = LaunchConfiguration(
        'robot_description_semantic_config_path')
    robot_description_kinematics_path = LaunchConfiguration(
        'robot_description_kinematics_path')
    robot_description_joints_limits_path = LaunchConfiguration(
        'robot_description_joints_limits_path')
    config_file_path = LaunchConfiguration(
        'config_file_path')
    config_name = LaunchConfiguration('config_name')
    results_dir_path = LaunchConfiguration('results_dir_path')

    robot_description = ParameterValue(Command(['xacro ',
                                                robot_description_file_path,
                                                ""]),  # you can add your xacro arguments here
                                       value_type=str)
    robot_description_semantic_config = load_file(
        robot_description_semantic_config_path.perform(context))
    kinematics_yaml = load_yaml(
        robot_description_kinematics_path.perform(context))
    joint_limits_yaml = load_yaml(
        robot_description_joints_limits_path.perform(context))

    return [Node(
            package='reach_ros',
            executable='reach_ros_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic_config,
                'robot_description_kinematics': kinematics_yaml,
                'robot_description_planning': joint_limits_yaml,
                'config_file': ParameterValue(config_file_path),
                'config_name': ParameterValue(config_name),
                'results_dir': ParameterValue(results_dir_path)
            }])]
