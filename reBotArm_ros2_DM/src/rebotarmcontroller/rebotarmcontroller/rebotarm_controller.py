from __future__ import annotations

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices


class reBotArmController(Node):
    def __init__(self) -> None:
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("arm_config", "")
        self.declare_parameter("gripper_config", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("safe_home_max_vel", 0.8)
        self.declare_parameter("gripper_assist_torque", 0.02)
        self.declare_parameter("gripper_assist_kd", 0.015)
        self.declare_parameter("gripper_assist_velocity_threshold", 0.08)
        self.declare_parameter("gripper_assist_velocity_full", 0.35)
        self.declare_parameter("gripper_assist_speed_limit", 0.8)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")

        arm_config = self.get_parameter("arm_config").value or None
        gripper_config = self.get_parameter("gripper_config").value or None
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        requested_safe_home_max_vel = float(
            self.get_parameter("safe_home_max_vel").value
        )
        safe_home_max_vel = max(0.1, min(requested_safe_home_max_vel, 1.5))
        if safe_home_max_vel != requested_safe_home_max_vel:
            self.get_logger().warn(
                "safe_home_max_vel must be in [0.1, 1.5] rad/s; "
                f"using {safe_home_max_vel:.2f}"
            )
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = HardwareManager(
            arm_cfg=arm_config,
            gripper_cfg=gripper_config,
            channel=channel,
            gripper_assist_torque=float(
                self.get_parameter("gripper_assist_torque").value
            ),
            gripper_assist_kd=float(
                self.get_parameter("gripper_assist_kd").value
            ),
            gripper_assist_velocity_threshold=float(
                self.get_parameter("gripper_assist_velocity_threshold").value
            ),
            gripper_assist_velocity_full=float(
                self.get_parameter("gripper_assist_velocity_full").value
            ),
            gripper_assist_speed_limit=float(
                self.get_parameter("gripper_assist_speed_limit").value
            ),
        )
        self.hardware.connect()

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(
            self,
            self.hardware,
            self.arm_namespace,
            safe_home_max_vel,
        )
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    def publish_arm_status(self) -> None:
        self.joint_state_publisher.publish_status()

    def shutdown(self) -> None:
        self.hardware.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = reBotArmController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
