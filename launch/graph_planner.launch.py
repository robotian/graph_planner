import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find package share directory
    pkg_share = get_package_share_directory('graph_planner')
    default_graph_file = os.path.join(pkg_share, 'data', 'graph_data.json')

    # Declare Launch Arguments for flexibility
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0921',
        description='Top-level namespace for the node'
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph_file_path',
        default_value=default_graph_file,
        description='Full path to the graph JSON file'
    )

    # Define Node configuration
    graph_planner_node = Node(
        package='graph_planner',
        executable='graph_planner_node',
        name='graph_planner_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'graph_file_path': LaunchConfiguration('graph_file_path'),
            'global_frame': 'map',
            'robot_base_frame': 'base_link'
        }],
        # Re-maps absolute TF topics (/tf, /tf_static) to the global root relative namespace
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(graph_planner_node)

    return ld