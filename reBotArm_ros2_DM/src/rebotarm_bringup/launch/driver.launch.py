from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    hardware_config = LaunchConfiguration("hardware_config")
    model = LaunchConfiguration("model")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    safe_home_max_vel = LaunchConfiguration("safe_home_max_vel")
    gripper_assist_torque = LaunchConfiguration("gripper_assist_torque")
    gripper_assist_kd = LaunchConfiguration("gripper_assist_kd")
    gripper_assist_velocity_threshold = LaunchConfiguration("gripper_assist_velocity_threshold")
    gripper_assist_velocity_full = LaunchConfiguration("gripper_assist_velocity_full")
    gripper_assist_speed_limit = LaunchConfiguration("gripper_assist_speed_limit")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    disable_after_safe_home = LaunchConfiguration("disable_after_safe_home")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "rebotarm_hardware.yaml"]
                ),
            ),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("safe_home_max_vel", default_value="0.8"),
            DeclareLaunchArgument("gripper_assist_torque", default_value="0.02"),
            DeclareLaunchArgument("gripper_assist_kd", default_value="0.015"),
            DeclareLaunchArgument("gripper_assist_velocity_threshold", default_value="0.08"),
            DeclareLaunchArgument("gripper_assist_velocity_full", default_value="0.35"),
            DeclareLaunchArgument("gripper_assist_speed_limit", default_value="0.8"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[
                    {
                        "hardware_config": hardware_config,
                        "model": model,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "safe_home_max_vel": safe_home_max_vel,
                        "gripper_assist_torque": gripper_assist_torque,
                        "gripper_assist_kd": gripper_assist_kd,
                        "gripper_assist_velocity_threshold": gripper_assist_velocity_threshold,
                        "gripper_assist_velocity_full": gripper_assist_velocity_full,
                        "gripper_assist_speed_limit": gripper_assist_speed_limit,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "disable_after_safe_home": ParameterValue(
                            disable_after_safe_home,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
