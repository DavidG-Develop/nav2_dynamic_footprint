"""Launch a plain or lifecycle footprint node, standalone or composed."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterValue


def start(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    urdf = argument('urdf')
    description = Path(urdf).read_text() if urdf else ''
    use_lifecycle = argument('use_lifecycle') == 'true'
    namespace = argument('namespace')
    parameters = [{
        'robot_description': ParameterValue(description, value_type=str),
        'robot_description_topic': argument('robot_description_topic'),
        'base_frame': argument('base_frame'),
        'footprint_topic': argument('footprint_topic'),
    }]
    if argument('use_composition') == 'false':
        executable = (
            'dynamic_footprint_lifecycle_node' if use_lifecycle else 'dynamic_footprint_node')
        return [Node(
            package='nav2_dynamic_footprint', executable=executable,
            name='dynamic_footprint', namespace=namespace, parameters=parameters, output='screen')]

    plugin = 'DynamicFootprintLifecycle' if use_lifecycle else 'DynamicFootprint'
    component = ComposableNode(
        package='nav2_dynamic_footprint', plugin=f'nav2_dynamic_footprint::{plugin}',
        name='dynamic_footprint', namespace=namespace, parameters=parameters)
    container = argument('container_name')
    if container:
        return [LoadComposableNodes(
            target_container=container, composable_node_descriptions=[component])]
    return [ComposableNodeContainer(
        package='rclcpp_components', executable='component_container',
        name='dynamic_footprint_container', namespace=namespace,
        composable_node_descriptions=[component], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_lifecycle', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_composition', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'container_name', default_value='',
            description='Existing container to load into; empty starts a dedicated container.'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('urdf', default_value=''),
        DeclareLaunchArgument('robot_description_topic', default_value='robot_description'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('footprint_topic', default_value='/dynamic_footprint'),
        OpaqueFunction(function=start),
    ])
