from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    safe_home_max_vel = LaunchConfiguration("safe_home_max_vel")
    gripper_assist_torque = LaunchConfiguration("gripper_assist_torque")
    gripper_assist_kd = LaunchConfiguration("gripper_assist_kd")
    gripper_assist_velocity_threshold = LaunchConfiguration("gripper_assist_velocity_threshold")
    gripper_assist_velocity_full = LaunchConfiguration("gripper_assist_velocity_full")
    gripper_assist_speed_limit = LaunchConfiguration("gripper_assist_speed_limit")
    gripper_assist_breakaway_fraction = LaunchConfiguration(
        "gripper_assist_breakaway_fraction"
    )
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("safe_home_max_vel", default_value="0.8"),
            DeclareLaunchArgument("gripper_assist_torque", default_value="0.04"),
            DeclareLaunchArgument("gripper_assist_kd", default_value="0.001"),
            DeclareLaunchArgument("gripper_assist_velocity_threshold", default_value="0.02"),
            DeclareLaunchArgument("gripper_assist_velocity_full", default_value="0.22"),
            DeclareLaunchArgument("gripper_assist_speed_limit", default_value="0.8"),
            DeclareLaunchArgument(
                "gripper_assist_breakaway_fraction",
                default_value="0.30",
            ),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[
                    {
                        "arm_config": arm_config,
                        "gripper_config": gripper_config,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "safe_home_max_vel": safe_home_max_vel,
                        "gripper_assist_torque": gripper_assist_torque,
                        "gripper_assist_kd": gripper_assist_kd,
                        "gripper_assist_velocity_threshold": gripper_assist_velocity_threshold,
                        "gripper_assist_velocity_full": gripper_assist_velocity_full,
                        "gripper_assist_speed_limit": gripper_assist_speed_limit,
                        "gripper_assist_breakaway_fraction": (
                            gripper_assist_breakaway_fraction
                        ),
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                    }
                ],
            ),
        ]
    )
