# launch/mpc_three_nodes.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import datetime


def generate_launch_description():
    # --- Perception->MPC inputs node args ---
    target_topic = LaunchConfiguration("target_topic")
    sk_topic = LaunchConfiguration("sk_topic")
    robot_pose_topic = LaunchConfiguration("robot_pose_topic")

    lat_vec_topic = LaunchConfiguration("lat_vec_topic")
    lat_scalar_topic = LaunchConfiguration("lat_scalar_topic")
    sn_topic = LaunchConfiguration("sn_topic")
    stab_topic = LaunchConfiguration("stab_topic")

    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    normal_step = LaunchConfiguration("normal_step")
    scalar_mode = LaunchConfiguration("scalar_mode")
    require_frame_match = LaunchConfiguration("require_frame_match")

    # --- MPC node args ---
    ctrl_rate_hz = LaunchConfiguration("ctrl_rate_hz")

    # --- Bridge args ---
    dxy_topic = LaunchConfiguration("dxy_topic")
    safety_topic = LaunchConfiguration("safety_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")

    cmd_rate_hz = LaunchConfiguration("cmd_rate_hz")
    max_step = LaunchConfiguration("max_step")
    deadband = LaunchConfiguration("deadband")
    scale = LaunchConfiguration("scale")

    # --- CSV logger args ---
    log_dir = LaunchConfiguration("log_dir")
    log_name = LaunchConfiguration("log_name")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    default_log_name = f"mpc_topics_{stamp}.csv"

    topics_to_log = [
        "/mpc/lateral_error_vec",
        "/mpc/lateral_error",
        "/robot_command",
        "/robot_current_pose",
        "/perception/sk",
        "/perception/target_point",
        "/mpc/dk_cmd",
        "/mpc/dxy_cmd",
    ]

    return LaunchDescription([
        # -------- Launch arguments --------
        DeclareLaunchArgument("target_topic", default_value="/perception/target_point"),
        DeclareLaunchArgument("sk_topic", default_value="/perception/sk"),
        DeclareLaunchArgument("robot_pose_topic", default_value="/robot_current_pose"),

        DeclareLaunchArgument("lat_vec_topic", default_value="/mpc/lateral_error_vec"),
        DeclareLaunchArgument("lat_scalar_topic", default_value="/mpc/lateral_error"),
        DeclareLaunchArgument("sn_topic", default_value="/mpc/normal_step"),
        DeclareLaunchArgument("stab_topic", default_value="/mpc/stability_index"),

        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("normal_step", default_value="0.001"),
        DeclareLaunchArgument("scalar_mode", default_value="xy_norm"),
        DeclareLaunchArgument("require_frame_match", default_value="false"),

        DeclareLaunchArgument("ctrl_rate_hz", default_value="20.0"),

        DeclareLaunchArgument("dxy_topic", default_value="/mpc/dxy_cmd"),
        DeclareLaunchArgument("safety_topic", default_value="/mpc/safety_mode"),
        DeclareLaunchArgument("cmd_topic", default_value="/robot_command"),

        DeclareLaunchArgument("cmd_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("max_step", default_value="0.002"),
        DeclareLaunchArgument("deadband", default_value="0.00001"),
        DeclareLaunchArgument("scale", default_value="1.0"),

        # -------- CSV logger arguments --------
        # 默认写到容器内 /tmp；如果你要主机可见，请把 log_dir 指到已挂载目录（如 /workspace/logs）
        DeclareLaunchArgument("log_dir", default_value="/tmp"),
        DeclareLaunchArgument("log_name", default_value=default_log_name),

        # -------- (0) CSV Topic Logger (starts immediately) --------
        Node(
            package="sk_mpc_controller",
            executable="csv_topic_logger",
            name="csv_topic_logger",
            output="screen",
            parameters=[{
                "topics": topics_to_log,
                "output_dir": log_dir,
                "filename": log_name,
                "discover_period_sec": 0.2,
            }],
        ),

        # -------- (1) perception -> mpc inputs (2D) --------
        Node(
            package="sk_mpc_controller",
            executable="perception_to_mpc_inputs_2d",
            name="perception_to_mpc_inputs_2d",
            output="screen",
            parameters=[{
                "target_topic": target_topic,
                "sk_topic": sk_topic,
                "robot_pose_topic": robot_pose_topic,

                "lat_vec_topic": lat_vec_topic,
                "lat_scalar_topic": lat_scalar_topic,
                "sn_topic": sn_topic,
                "stab_topic": stab_topic,

                "publish_rate_hz": publish_rate_hz,
                "normal_step": normal_step,
                "scalar_mode": scalar_mode,
                "require_frame_match": require_frame_match,
            }],
        ),

        # -------- (2) MPC core (2D) --------
        Node(
            package="sk_mpc_controller",
            executable="mpc_node_2d",
            name="mpc_node_2d",
            output="screen",
            parameters=[{
                "ctrl_rate_hz": ctrl_rate_hz,
            }],
        ),

        # -------- (3) Bridge (2D) --------
        Node(
            package="sk_mpc_controller",
            executable="mpc_to_robot_command_bridge_2d",
            name="mpc_to_robot_command_bridge_2d",
            output="screen",
            parameters=[{
                "dxy_topic": dxy_topic,
                "safety_topic": safety_topic,
                "robot_pose_topic": robot_pose_topic,
                "cmd_topic": cmd_topic,

                "cmd_rate_hz": cmd_rate_hz,
                "max_step": max_step,
                "deadband": deadband,
                "scale": scale,
            }],
        ),
    ])

