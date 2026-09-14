import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('articubot_one')
    world = os.path.join(pkg_share, 'worlds', 'slam_test.sdf')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_sim.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'online_async_launch.py')
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    frontier = Node(
        package='articubot_one',
        executable='frontier_explorer.py',
        output='screen'
    )

    # The stack is started in stages rather than all at once: SLAM Toolbox needs
    # /scan and /clock to exist before it will publish map->odom, Nav2's costmaps
    # need that transform before they will activate, and the explorer needs the
    # navigate_to_pose action server up before it sends its first goal.
    return LaunchDescription([
        sim,
        TimerAction(period=8.0, actions=[slam]),
        TimerAction(period=10.0, actions=[nav2]),
        TimerAction(period=12.0, actions=[frontier]),
    ])
