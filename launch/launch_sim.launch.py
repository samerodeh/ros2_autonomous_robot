import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    package_name = "articubot_one"
    pkg_share = get_package_share_directory(package_name)

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_share, "worlds", "empty_world.sdf"),
        description="Gazebo world file",
    )
    world = LaunchConfiguration("world")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Robot description
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(pkg_share, "description", "robot.urdf.xacro"),
            " use_ros2_control:=false",
            " sim_mode:=true",
        ]),
        value_type=str,
    )

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # Gazebo Sim, server-only, wrapped in xvfb-run so the sensor plugins still have
    # an X display to render against inside a headless container.
    gazebo = ExecuteProcess(
        cmd=["xvfb-run", "-a", "ign", "gazebo", "-r", "-v", "4", "-s", world],
        output="screen",
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "articubot",
            "-z", "0.1",
        ],
        output="screen",
    )

    # Bridge - connects Gazebo transport topics to ROS 2 topics.
    # `[` is Gazebo -> ROS, `]` is ROS -> Gazebo.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # SLAM Toolbox and the Nav2 costmaps both feed off /scan.
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gazebo,
        rsp,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=7.0, actions=[bridge]),
    ])
