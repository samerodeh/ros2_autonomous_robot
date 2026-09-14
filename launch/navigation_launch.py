import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('articubot_one')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    params = ParameterFile(params_file, allow_substs=True)
    # Applied on top of nav2_params.yaml for the controller server only: the DWB
    # goal checker and footprint values that must win over the upstream defaults.
    dwb_override = os.path.join(bringup_dir, 'config', 'dwb_critics_override.yaml')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('default_bt_xml_filename', default_value=os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')),
        DeclareLaunchArgument('map_subscribe_transient_local', default_value='false'),
        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[params, dwb_override], remappings=remappings),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[params], remappings=remappings),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen', parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart}, {'node_names': lifecycle_nodes}]),
    ])
