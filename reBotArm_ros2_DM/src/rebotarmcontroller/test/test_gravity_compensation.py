from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rebotarmcontroller.hardware_manager import HardwareManager  # noqa: E402


def _fake_kinematics_modules(observed_q: list[np.ndarray]):
    package = ModuleType("reBotArm_control_py")
    kinematics = ModuleType("reBotArm_control_py.kinematics")

    def pad_q_for_model(model, q, controlled_joints):
        del controlled_joints
        padded = np.zeros(model.nq, dtype=np.float64)
        padded[: len(q)] = q
        observed_q.append(padded.copy())
        return padded

    kinematics.pad_q_for_model = pad_q_for_model
    package.kinematics = kinematics
    return {
        "reBotArm_control_py": package,
        "reBotArm_control_py.kinematics": kinematics,
    }


class _FakeArmGroup:
    def __init__(self, positions: np.ndarray) -> None:
        self.positions = positions.copy()
        self.num_joints = len(positions)
        self._mit_kp = np.full(self.num_joints, 10.0)
        self._mit_kd = np.full(self.num_joints, 2.0)
        self.sent = None
        self.mode_pos_vel_calls = 0

    def get_positions(self, request_feedback: bool = True) -> np.ndarray:
        del request_feedback
        return self.positions.copy()

    def send_mit(self, **command) -> None:
        self.sent = command

    def enable(self) -> None:
        pass

    def mode_pos_vel(self) -> bool:
        self.mode_pos_vel_calls += 1
        return True


class GravityCompensationTests(unittest.TestCase):
    def test_gripper_assist_is_zero_at_rest_limits_and_high_speed(self) -> None:
        manager = HardwareManager.__new__(HardwareManager)
        manager._gripper_assist_velocity_threshold = 0.08
        manager._gripper_assist_velocity_full = 0.35
        manager._gripper_assist_speed_limit = 0.8
        manager._gripper_assist_torque = 0.02

        self.assertEqual(manager._gripper_assist_feedforward(-2.5, 0.02), 0.0)
        self.assertEqual(manager._gripper_assist_feedforward(-0.1, 0.2), 0.0)
        self.assertEqual(manager._gripper_assist_feedforward(-4.9, -0.2), 0.0)
        self.assertEqual(manager._gripper_assist_feedforward(-2.5, 0.9), 0.0)
        self.assertGreater(manager._gripper_assist_feedforward(-2.5, 0.2), 0.0)
        self.assertLess(manager._gripper_assist_feedforward(-2.5, -0.2), 0.0)
        self.assertAlmostEqual(
            manager._gripper_assist_feedforward(-2.5, 0.35),
            0.02,
        )

    def test_start_gripper_assist_enters_limited_mit_mode(self) -> None:
        calls = []
        manager = HardwareManager.__new__(HardwareManager)
        manager._gripper_mot = SimpleNamespace(
            enable=lambda: calls.append("enable"),
            ensure_mode=lambda mode, timeout: calls.append(("mode", mode, timeout)),
        )
        manager._gripper_lock = threading.Lock()
        manager._gripper_active = True
        manager._gripper_manual_free = True
        manager._gripper_assist_active = False
        manager._gripper_assist_velocity = 1.0
        manager._gripper_tick = lambda: calls.append("feedback")
        motorbridge = ModuleType("motorbridge")
        motorbridge.Mode = SimpleNamespace(MIT="mit")

        with patch.dict(sys.modules, {"motorbridge": motorbridge}):
            manager.start_gripper_assist()

        self.assertEqual(calls[0:2], ["feedback", "enable"])
        self.assertIn(("mode", "mit", 1000), calls)
        self.assertTrue(manager.gripper_assist_active())
        self.assertFalse(manager.gripper_manual_free())
        self.assertEqual(manager._gripper_assist_velocity, 0.0)

    def test_release_gripper_disables_only_gripper_motor(self) -> None:
        calls = []
        manager = HardwareManager.__new__(HardwareManager)
        manager._gripper_mot = SimpleNamespace(
            disable=lambda: calls.append("disable")
        )
        manager._gripper_manual_free = False
        manager._gripper_active = True
        manager._gripper_lock = threading.Lock()
        manager._gripper_tick = lambda: calls.append("feedback")

        manager.release_gripper_for_manual()

        self.assertEqual(calls, ["feedback", "disable"])
        self.assertTrue(manager.gripper_manual_free())
        self.assertFalse(manager._gripper_active)

    def test_hold_gripper_uses_measured_position_without_jump(self) -> None:
        calls = []
        motor = SimpleNamespace(
            enable=lambda: calls.append("enable"),
            ensure_mode=lambda mode, timeout: calls.append(("mode", mode, timeout)),
            send_pos_vel=lambda position, vlim: calls.append(("send", position, vlim)),
        )
        manager = HardwareManager.__new__(HardwareManager)
        manager._gripper_mot = motor
        manager._gripper_cfg = SimpleNamespace(vlim=2.5)
        manager._gripper_manual_free = True
        manager._gripper_active = False
        manager._gripper_target_angle = 0.0
        manager._gripper_pos = -2.25
        manager._gripper_lock = threading.Lock()
        manager._gripper_tick = lambda: calls.append("feedback")
        motorbridge = ModuleType("motorbridge")
        motorbridge.Mode = SimpleNamespace(POS_VEL="pos_vel")

        with patch.dict(sys.modules, {"motorbridge": motorbridge}):
            manager.hold_gripper_current()

        self.assertEqual(calls[0:2], ["feedback", "enable"])
        self.assertIn(("mode", "pos_vel", 1000), calls)
        self.assertIn(("send", -2.25, 2.5), calls)
        self.assertEqual(manager._gripper_target_angle, -2.25)
        self.assertFalse(manager.gripper_manual_free())

    def test_safe_home_preserves_gripper_state(self) -> None:
        manager = HardwareManager.__new__(HardwareManager)
        safe_home_calls = []
        manager._endpos_ctrl = SimpleNamespace(
            safe_home=lambda **kwargs: safe_home_calls.append(kwargs)
        )
        manager._gripper_mot = object()
        manager._state_machine = "TRAJ_RUNNING"
        manager.set_gripper_position = lambda *args, **kwargs: self.fail(
            "safe_home must not command the gripper"
        )

        manager.safe_home(max_vel=0.8)

        self.assertEqual(safe_home_calls, [{"max_vel": 0.8}])
        self.assertEqual(manager.state_machine, "IDLE")

    def test_config_vector_accepts_scalar_and_rejects_wrong_length(self) -> None:
        np.testing.assert_allclose(
            HardwareManager._config_vector(1.5, 6, "gain", 0.0),
            np.full(6, 1.5),
        )
        with self.assertRaisesRegex(ValueError, "scalar or 6 values"):
            HardwareManager._config_vector([1.0, 2.0], 6, "gain", 0.0)

    def test_gravity_torque_accepts_six_axis_state_with_eight_dof_model(self) -> None:
        manager = HardwareManager.__new__(HardwareManager)
        manager._gc_model = SimpleNamespace(nq=8)
        manager._gc_data = object()
        manager._gc_compute_generalized_gravity = lambda model, q, data: np.zeros(
            model.nq
        )
        manager._gc_joint_direction = np.ones(6)
        manager._gc_tau_scale = np.ones(6)
        manager._gc_torque_limit = np.zeros(6)
        observed_q = []

        self.assertEqual(manager._gc_model.nq, 8)
        with patch.dict(sys.modules, _fake_kinematics_modules(observed_q)):
            tau = manager._gravity_torque(np.zeros(6))
        self.assertEqual(tau.shape, (6,))
        self.assertTrue(np.all(np.isfinite(tau)))
        self.assertEqual(observed_q[0].shape, (8,))

    def test_gravity_torque_applies_direction_scale_and_limit(self) -> None:
        manager = HardwareManager.__new__(HardwareManager)
        manager._gc_model = SimpleNamespace(nq=8)
        manager._gc_data = object()
        manager._gc_joint_direction = np.array([1, -1, 1, -1, 1, -1], dtype=float)
        manager._gc_tau_scale = np.array([1, 2, 1, 2, 1, 2], dtype=float)
        manager._gc_torque_limit = np.array([0, 3, 0, 5, 0, 7], dtype=float)
        observed_q = []

        def fake_gravity(model, q, data):
            del model, data
            observed_q.append(q.copy())
            return np.arange(1, 9, dtype=np.float64)

        manager._gc_compute_generalized_gravity = fake_gravity
        q = np.arange(1, 7, dtype=np.float64)
        with patch.dict(sys.modules, _fake_kinematics_modules(observed_q)):
            tau = manager._gravity_torque(q)

        np.testing.assert_allclose(observed_q[0][:6], q * manager._gc_joint_direction)
        np.testing.assert_allclose(tau, [1, -3, 3, -5, 5, -7])

    def test_tick_tracks_measured_pose_without_integral_or_jacobian(self) -> None:
        measured = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        group = _FakeArmGroup(measured)
        arm = SimpleNamespace(arm=group)
        manager = HardwareManager.__new__(HardwareManager)
        manager._arm = arm
        manager._gravity_comp_active = True
        manager._gravity_comp_q_target = np.zeros(6)
        manager._gravity_comp_q_last = measured.copy()
        manager._gravity_comp_transition_q_hold = None
        manager._gravity_comp_transition_started_at = None
        manager._gravity_comp_hold_kp = None
        manager._gravity_comp_hold_kd = None
        manager._gc_kp = np.full(6, 1.5)
        manager._gc_kd = np.full(6, 1.0)
        manager._gc_transition_duration = 0.5
        manager._gravity_torque = lambda q: np.arange(6, dtype=np.float64)

        manager._gravity_comp_tick(arm, 0.002)

        np.testing.assert_allclose(group.sent["pos"], measured)
        np.testing.assert_allclose(group.sent["kp"], manager._gc_kp)
        np.testing.assert_allclose(group.sent["kd"], manager._gc_kd)
        np.testing.assert_allclose(group.sent["tau"], np.arange(6))

    def test_startup_failure_restores_pos_vel_hold(self) -> None:
        measured = np.linspace(0.1, 0.6, 6)
        group = _FakeArmGroup(measured)
        arm = SimpleNamespace(arm=group, _rate=500.0)
        manager = HardwareManager.__new__(HardwareManager)
        manager._arm = arm
        manager._endpos_ctrl = SimpleNamespace(
            _stop_send=SimpleNamespace(set=lambda: None),
            _moving=True,
        )
        manager._enabled = True
        manager._gravity_comp_active = False
        manager._gravity_comp_q_target = None
        manager._gravity_comp_q_last = None
        manager._gravity_comp_transition_q_hold = None
        manager._gravity_comp_transition_started_at = None
        manager._gravity_comp_hold_kp = None
        manager._gravity_comp_hold_kd = None
        manager._gravity_comp_fault = ""
        manager._error_codes = []
        manager._state_machine = "IDLE"
        manager._stop_control_loop = lambda: None
        manager._gravity_torque = lambda q: np.zeros(6)
        manager._enter_gravity_compensation_mode = lambda *args: None
        manager._gravity_comp_tick = lambda *args: (_ for _ in ()).throw(
            ValueError("expected 8, got 6")
        )
        restored_targets = []
        manager._start_pos_vel_loop = lambda target=None: restored_targets.append(
            target.copy()
        )

        with self.assertRaisesRegex(RuntimeError, "restored POS_VEL"):
            manager.start_gravity_compensation()

        self.assertFalse(manager.gravity_compensation_active())
        self.assertEqual(group.mode_pos_vel_calls, 1)
        np.testing.assert_allclose(restored_targets[0], measured)
        self.assertIn("expected 8, got 6", manager.gravity_compensation_fault())

    def test_gravity_start_enables_gripper_assist_by_default(self) -> None:
        measured = np.linspace(0.1, 0.6, 6)
        group = _FakeArmGroup(measured)
        loop_calls = []
        arm = SimpleNamespace(
            arm=group,
            _rate=500.0,
            start_control_loop=lambda callback, rate: loop_calls.append((callback, rate)),
        )
        manager = HardwareManager.__new__(HardwareManager)
        manager._arm = arm
        manager._endpos_ctrl = SimpleNamespace(
            _stop_send=SimpleNamespace(set=lambda: None),
            _moving=True,
        )
        manager._enabled = True
        manager._gravity_comp_active = False
        manager._gravity_comp_q_target = None
        manager._gravity_comp_q_last = None
        manager._gravity_comp_transition_q_hold = None
        manager._gravity_comp_transition_started_at = None
        manager._gravity_comp_hold_kp = None
        manager._gravity_comp_hold_kd = None
        manager._gravity_comp_fault = ""
        manager._error_codes = []
        manager._state_machine = "IDLE"
        manager._gripper_mot = None
        manager._stop_control_loop = lambda: None
        manager._gravity_torque = lambda q: np.zeros(6)
        manager._enter_gravity_compensation_mode = lambda *args: None
        manager._gravity_comp_tick = lambda *args: None
        assist_calls = []
        manager.start_gripper_assist = lambda: assist_calls.append(True)
        manager.release_gripper_for_manual = lambda: self.fail(
            "gravity compensation must default to low-resistance assist"
        )

        manager.start_gravity_compensation()

        self.assertEqual(assist_calls, [True])
        self.assertEqual(manager.state_machine, "GRAVITY_COMP")
        self.assertTrue(manager.gravity_compensation_active())
        self.assertEqual(len(loop_calls), 1)


if __name__ == "__main__":
    unittest.main()
