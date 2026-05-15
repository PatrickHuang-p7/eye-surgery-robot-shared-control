import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # pragma: no cover - 兼容未暴露该接口的运行环境
    SignalHandlerOptions = None

from .kinematics_solver import KinematicsSolver


class State:
    INIT = -1
    HOME = 0
    TRANSIT = 1
    HOVER = 2
    GRASP = 3
    PINCH = 4
    SHARED_CONTROL = 5
    RETURN_RELEASE = 6
    RETURN_HOVER = 7
    RETURN_TRANSIT = 8
    RETURN_HOME = 9
    SAFE_HOLD = 10
    REGRASP = 11


class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.kinematics = KinematicsSolver()
        self.current_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        self.state = State.INIT

        # ---------------------------
        # 轨迹点与抓取姿态
        # ---------------------------
        self.home_xyz = np.array([0.20, 0.0, 0.18], dtype=float)
        self.transit_xyz = np.array([0.22, 0.0, 0.18], dtype=float)
        self.hover_xyz = np.array([0.22, 0.0, 0.10], dtype=float)
        self.grasp_xyz = np.array([0.22, 0.0, 0.08], dtype=float)

        self.target_phi = 0.0
        self.gripper_open_angle = 1.0
        self.gripper_close_angle = 0.0

        self.q_home = None
        self.q_transit = None
        self.q_hover = None
        self.q_grasp = None

        # ---------------------------
        # ROS 参数与控制增益
        # ---------------------------
        self.declare_parameter('joystick_follow_scale_x_m', 0.009)
        self.declare_parameter('joystick_follow_scale_y_m', 0.009)
        self.declare_parameter('virtual_force_gain_x_m_per_n', 0.0009)
        self.declare_parameter('virtual_force_gain_y_m_per_n', 0.0009)
        self.declare_parameter('authority_force_start_n', 1.0)
        self.declare_parameter('authority_force_full_n', 4.0)
        self.declare_parameter('shared_control_height_m', 0.08)
        self.declare_parameter('max_joystick_offset_x_m', 0.008)
        self.declare_parameter('max_joystick_offset_y_m', 0.008)
        self.declare_parameter('max_virtual_correction_x_m', 0.008)
        self.declare_parameter('max_virtual_correction_y_m', 0.008)
        self.declare_parameter('smooth_alpha', 0.06)
        self.declare_parameter('gripper_contact_required', True)
        self.declare_parameter('gripper_relief_angle', 0.15)
        self.declare_parameter('pinch_contact_hold_s', 0.50)
        self.declare_parameter('pinch_close_rate_per_s', 0.60)
        self.declare_parameter('grip_hold_open_offset', 0.03)
        self.declare_parameter('use_fixed_grip_hold_angle', False)
        self.declare_parameter('fixed_grip_hold_angle', 0.18)
        self.declare_parameter('shared_contact_loss_timeout_s', 0.60)
        self.declare_parameter('return_home_on_shutdown', True)
        self.declare_parameter('shutdown_step_duration_s', 1.20)
        self.declare_parameter('shutdown_spin_dt_s', 0.05)
        self.declare_parameter('regrasp_relief_duration_s', 0.5)
        self.declare_parameter('regrasp_max_attempts', 3)
        self.declare_parameter('shared_idle_open_extra_m', 0.010)
        self.declare_parameter('shared_sub_state_enter_dist', 0.9)
        self.declare_parameter('shared_sub_state_exit_dist', 0.7)
        self.declare_parameter('shared_recenter_complete_dist', 0.05)
        self.declare_parameter('shared_control_lock_state', True)
        self.declare_parameter('shared_control_bypass_fsr', False)
        self.declare_parameter('laser_recenter_gain', 0.85)

        self.joystick_follow_scale_xy = np.array([
            max(0.0, float(self.get_parameter('joystick_follow_scale_x_m').value)),
            max(0.0, float(self.get_parameter('joystick_follow_scale_y_m').value)),
        ], dtype=float)
        self.virtual_force_gain_xy = np.array([
            float(self.get_parameter('virtual_force_gain_x_m_per_n').value),
            float(self.get_parameter('virtual_force_gain_y_m_per_n').value),
        ], dtype=float)
        self.authority_force_start_n = max(
            0.0,
            float(self.get_parameter('authority_force_start_n').value),
        )
        self.authority_force_full_n = max(
            self.authority_force_start_n + 1e-6,
            float(self.get_parameter('authority_force_full_n').value),
        )
        self.shared_control_height_m = float(
            self.get_parameter('shared_control_height_m').value
        )
        self.max_joystick_offset_xy = np.array([
            max(1e-4, float(self.get_parameter('max_joystick_offset_x_m').value)),
            max(1e-4, float(self.get_parameter('max_joystick_offset_y_m').value)),
        ], dtype=float)
        self.max_virtual_correction_xy = np.array([
            max(0.0, float(self.get_parameter('max_virtual_correction_x_m').value)),
            max(0.0, float(self.get_parameter('max_virtual_correction_y_m').value)),
        ], dtype=float)
        self.gripper_contact_required = bool(
            self.get_parameter('gripper_contact_required').value
        )
        self.gripper_relief_angle = float(self.get_parameter('gripper_relief_angle').value)
        self.pinch_contact_hold_s = max(
            0.0,
            float(self.get_parameter('pinch_contact_hold_s').value),
        )
        self.pinch_close_rate_per_s = max(
            1e-4,
            float(self.get_parameter('pinch_close_rate_per_s').value),
        )
        self.grip_hold_open_offset = max(
            0.0,
            float(self.get_parameter('grip_hold_open_offset').value),
        )
        self.use_fixed_grip_hold_angle = bool(
            self.get_parameter('use_fixed_grip_hold_angle').value
        )
        self.fixed_grip_hold_angle = self._clamp_gripper_angle(
            float(self.get_parameter('fixed_grip_hold_angle').value)
        )
        self.shared_contact_loss_timeout_s = max(
            0.0,
            float(self.get_parameter('shared_contact_loss_timeout_s').value),
        )
        self.return_home_on_shutdown = bool(
            self.get_parameter('return_home_on_shutdown').value
        )
        self.shutdown_step_duration_s = max(
            0.2,
            float(self.get_parameter('shutdown_step_duration_s').value),
        )
        self.shutdown_spin_dt_s = max(
            0.01,
            float(self.get_parameter('shutdown_spin_dt_s').value),
        )
        self.regrasp_relief_duration_s = max(
            0.1,
            float(self.get_parameter('regrasp_relief_duration_s').value),
        )
        self.regrasp_max_attempts = max(
            1,
            int(self.get_parameter('regrasp_max_attempts').value),
        )
        self.shared_idle_open_extra_m = max(
            0.0,
            float(self.get_parameter('shared_idle_open_extra_m').value),
        )
        self.shared_sub_state_enter_dist = max(
            0.0, float(self.get_parameter('shared_sub_state_enter_dist').value)
        )
        self.shared_sub_state_exit_dist = max(
            0.0, float(self.get_parameter('shared_sub_state_exit_dist').value)
        )
        self.shared_recenter_complete_dist = max(
            0.0, float(self.get_parameter('shared_recenter_complete_dist').value)
        )
        self.shared_control_lock_state = bool(
            self.get_parameter('shared_control_lock_state').value
        )
        self.shared_control_bypass_fsr = bool(
            self.get_parameter('shared_control_bypass_fsr').value
        )
        self.laser_recenter_gain = max(
            0.0, float(self.get_parameter('laser_recenter_gain').value)
        )

        # ---------------------------
        # ROS 2 接口
        # ---------------------------
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/arm/joint_command', 10)
        self.shared_feedback_pub = self.create_publisher(
            Float32MultiArray, '/shared_control_feedback', 10
        )
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Float32MultiArray, '/joy_position', self.joy_cb, 10)
        self.create_subscription(
            Float32MultiArray,
            '/virtual_fixture_force',
            self.virtual_force_cb,
            10,
        )
        self.create_subscription(Bool, '/gripper_contact', self.gripper_contact_cb, 10)
        self.create_subscription(
            Bool,
            '/gripper_overpressure',
            self.gripper_overpressure_cb,
            10,
        )
        self.create_subscription(
            Bool,
            '/gripper_sensor_fault',
            self.gripper_sensor_fault_cb,
            10,
        )
        self.create_subscription(Bool, '/arm_control_return_home', self.return_home_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/laser_position', self.laser_position_cb, 10
        )

        # ---------------------------
        # 控制器内部状态
        # ---------------------------
        self.joy_xy = np.array([0.0, 0.0], dtype=float)
        self.joy_vxy = np.array([0.0, 0.0], dtype=float)
        self.virtual_force = np.array([0.0, 0.0, 0.0], dtype=float)
        self.net_force = np.array([0.0, 0.0, 0.0], dtype=float)
        self.gripper_contact_ok = False
        self.gripper_overpressure = False
        self.gripper_sensor_fault = False
        self.authority_alpha = 0.0

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.smooth_q = None
        self.alpha = max(0.01, min(1.0, float(self.get_parameter('smooth_alpha').value)))
        self.state_timer = 0.0
        self.active_xyz = self.grasp_xyz.copy()
        self.shared_offset_xy = np.zeros(2, dtype=float)
        self.current_gripper_cmd = self.gripper_open_angle
        self.shared_gripper_hold_angle = self.gripper_open_angle
        self.pinch_contact_start_time = None
        self.contact_detected_gripper_angle = None
        self.shared_contact_loss_start_time = None
        self.regrasp_attempt_count = 0
        self.laser_dist_norm = 0.0
        self.laser_xy_norm = np.zeros(2, dtype=float)
        self.laser_safe_radius_norm = 1.0
        self.laser_radius_px = 0.0
        self.laser_safe_radius_px = 0.0
        self.laser_boundary_radius_px = 0.0
        self.shared_sub_state = 1   # 1=人手主导，2=机器主导
        self.return_home_requested = False
        self.return_after_release_state = State.RETURN_HOME
        self.return_release_arm_q = None
        self.grasp_xyz[2] = self.shared_control_height_m
        self.active_xyz[2] = self.shared_control_height_m

        self.get_logger().info(
            '机械臂共享控制节点启动：摇杆位置跟随 + 虚拟边界回推 + FSR夹持保护。'
        )
        self._log_shared_control_limits()
        if self.use_fixed_grip_hold_angle:
            self.get_logger().info(
                f'夹持角策略: 固定持握角模式 | FixedGripHoldAngle={self.fixed_grip_hold_angle:.3f}'
            )
        else:
            self.get_logger().info(
                f'夹持角策略: 接触后微开模式 | GripHoldOpenOffset={self.grip_hold_open_offset:.3f}'
            )
        if self.shared_control_bypass_fsr:
            self.get_logger().warn(
                '共享控制调参模式已开启：SHARED_CONTROL 阶段将忽略 FSR 的接触/过压/故障保护，仅保留摇杆与虚拟边界逻辑。'
            )

    def joint_cb(self, msg: JointState) -> None:
        if len(msg.position) == 5:
            self.current_q = np.array(msg.position, dtype=float)

    def joy_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data[:4])
        while len(data) < 4:
            data.append(0.0)
        self.joy_xy = np.array(data[:2], dtype=float)
        self.joy_vxy = np.array(data[2:4], dtype=float)

    def virtual_force_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data[:3])
        while len(data) < 3:
            data.append(0.0)
        self.virtual_force = np.array(data, dtype=float)

    def gripper_contact_cb(self, msg: Bool) -> None:
        self.gripper_contact_ok = bool(msg.data)

    def gripper_overpressure_cb(self, msg: Bool) -> None:
        self.gripper_overpressure = bool(msg.data)

    def gripper_sensor_fault_cb(self, msg: Bool) -> None:
        self.gripper_sensor_fault = bool(msg.data)

    def laser_position_cb(self, msg: Float32MultiArray) -> None:
        data = list(msg.data[:7])
        while len(data) < 7:
            data.append(0.0)
        self.laser_xy_norm = np.array(data[:2], dtype=float)
        self.laser_dist_norm = float(data[2])
        self.laser_safe_radius_norm = max(0.0, float(data[3]))
        self.laser_radius_px = max(0.0, float(data[4]))
        self.laser_safe_radius_px = max(0.0, float(data[5]))
        self.laser_boundary_radius_px = max(0.0, float(data[6]))

    def return_home_cb(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        self.return_home_requested = True
        self.get_logger().warn('收到手动回原始安全位指令，将在控制循环中执行安全回撤。')

    def _compute_target_offset_xy(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, bool]:
        desired_offset_xy = self.joy_xy * self.joystick_follow_scale_xy
        desired_offset_xy = np.clip(
            desired_offset_xy,
            -self.max_joystick_offset_xy,
            self.max_joystick_offset_xy,
        )

        virtual_correction_xy = self.virtual_force[:2] * self.virtual_force_gain_xy
        virtual_correction_xy = np.clip(
            virtual_correction_xy,
            -self.max_virtual_correction_xy,
            self.max_virtual_correction_xy,
        )

        # 显式人机控制权重：
        # alpha=0 时由人完全主导，alpha=1 时由安全修正完全接管。
        virtual_force_norm = float(np.linalg.norm(self.virtual_force[:2]))
        authority_alpha = (virtual_force_norm - self.authority_force_start_n) / (
            self.authority_force_full_n - self.authority_force_start_n
        )
        authority_alpha = max(0.0, min(1.0, authority_alpha))

        laser_recenter_xy = np.zeros(2, dtype=float)
        hard_recenter_active = False
        if self.laser_dist_norm > 1.0 + 1e-6:
            hard_recenter_active = True
            outside_ratio = max(0.0, min(1.0, self.laser_dist_norm - 1.0))
            hard_push_ratio = max(
                0.0,
                min(1.0, self.laser_recenter_gain + outside_ratio),
            )
            unit_xy = self.laser_xy_norm / max(np.linalg.norm(self.laser_xy_norm), 1e-9)
            laser_recenter_xy = (
                -unit_xy
                * self.max_joystick_offset_xy
                * hard_push_ratio
            )
            safety_target_xy = laser_recenter_xy
            target_offset_xy = safety_target_xy
            authority_alpha = 1.0
        else:
            safety_target_xy = np.clip(
                virtual_correction_xy,
                -self.max_joystick_offset_xy,
                self.max_joystick_offset_xy,
            )
            target_offset_xy = (
                (1.0 - authority_alpha) * desired_offset_xy
                + authority_alpha * safety_target_xy
            )
            target_offset_xy = np.clip(
                target_offset_xy,
                -self.max_joystick_offset_xy,
                self.max_joystick_offset_xy,
            )
        return (
            desired_offset_xy,
            virtual_correction_xy,
            laser_recenter_xy,
            safety_target_xy,
            target_offset_xy,
            authority_alpha,
            hard_recenter_active,
        )

    def _log_shared_control_limits(self) -> None:
        x_min = self.grasp_xyz[0] - self.max_joystick_offset_xy[0]
        x_max = self.grasp_xyz[0] + self.max_joystick_offset_xy[0]
        y_min = self.grasp_xyz[1] - self.max_joystick_offset_xy[1]
        y_max = self.grasp_xyz[1] + self.max_joystick_offset_xy[1]
        self.get_logger().info(
            (
                '共享控制约束: '
                f'Z固定={self.shared_control_height_m:.3f}m | '
                f'中心=({self.grasp_xyz[0]:.3f}, {self.grasp_xyz[1]:.3f})m | '
                f'X范围=[{x_min:.3f}, {x_max:.3f}]m | '
                f'Y范围=[{y_min:.3f}, {y_max:.3f}]m | '
                f'权重切换=[{self.authority_force_start_n:.2f}, {self.authority_force_full_n:.2f}]N'
            )
        )

    def _clamp_gripper_angle(self, angle: float) -> float:
        lower = min(self.gripper_close_angle, self.gripper_open_angle)
        upper = max(self.gripper_close_angle, self.gripper_open_angle)
        return max(lower, min(upper, angle))

    def _ensure_named_waypoints(self) -> bool:
        if self.q_home is None:
            self.q_home = self.kinematics.inverse_kinematics(self.home_xyz, self.target_phi)
        if self.q_transit is None:
            self.q_transit = self.kinematics.inverse_kinematics(self.transit_xyz, self.target_phi)
        if self.q_hover is None:
            self.q_hover = self.kinematics.inverse_kinematics(self.hover_xyz, self.target_phi)
        if self.q_grasp is None:
            self.q_grasp = self.kinematics.inverse_kinematics(self.grasp_xyz, self.target_phi)
        return all(q is not None for q in [self.q_home, self.q_transit, self.q_hover, self.q_grasp])

    def _get_hold_arm_angles(self) -> np.ndarray:
        if self.smooth_q is not None:
            return np.array(self.smooth_q, dtype=float)
        if np.any(self.current_q[:4]):
            return np.array(self.current_q[:4], dtype=float)
        if self.q_home is not None:
            return np.array(self.q_home, dtype=float)
        return np.zeros(4, dtype=float)

    def _select_grip_hold_angle(self, contact_angle: float) -> tuple[float, str, str]:
        contact_angle = self._clamp_gripper_angle(contact_angle)

        if self.use_fixed_grip_hold_angle:
            requested_hold_angle = self._clamp_gripper_angle(self.fixed_grip_hold_angle)
            selected_hold_angle = max(contact_angle, requested_hold_angle)
            note = ''
            if selected_hold_angle > requested_hold_angle + 1e-6:
                note = ' | 固定角度比接触角更紧，已按接触角限幅'
            return selected_hold_angle, 'fixed', note

        requested_hold_angle = self._clamp_gripper_angle(contact_angle + self.grip_hold_open_offset)
        selected_hold_angle = max(contact_angle, requested_hold_angle)
        return selected_hold_angle, 'contact_offset', ''

    def _run_shutdown_segment(
        self,
        target_arm_angles,
        target_gripper_angle: float,
        segment_name: str,
    ) -> None:
        segment_steps = max(
            1,
            int(round(self.shutdown_step_duration_s / self.shutdown_spin_dt_s)),
        )
        self.get_logger().info(
            f'退出控制流程: {segment_name}，持续发送安全目标 {self.shutdown_step_duration_s:.2f}s。'
        )
        for _ in range(segment_steps):
            if not rclpy.ok():
                break
            rclpy.spin_once(self, timeout_sec=0.0)
            self.smooth_and_publish(target_arm_angles, target_gripper_angle)
            time.sleep(self.shutdown_spin_dt_s)

    def _start_return_home_sequence(self, reason: str) -> None:
        if not self._ensure_named_waypoints():
            self.get_logger().warn('回原位失败：存在不可达的安全位。')
            self.return_home_requested = False
            return

        self.return_home_requested = False
        self.pinch_contact_start_time = None
        self.contact_detected_gripper_angle = None
        self.shared_contact_loss_start_time = None
        self.regrasp_attempt_count = 0

        if self.state in (
            State.RETURN_RELEASE,
            State.RETURN_HOVER,
            State.RETURN_TRANSIT,
            State.RETURN_HOME,
            State.SAFE_HOLD,
        ):
            return

        if self.state in (State.GRASP, State.PINCH, State.SHARED_CONTROL):
            self.return_after_release_state = State.RETURN_HOVER
        elif self.state == State.HOVER:
            self.return_after_release_state = State.RETURN_TRANSIT
        elif self.state in (State.TRANSIT, State.HOME):
            self.return_after_release_state = State.RETURN_HOME
        else:
            self.return_after_release_state = State.RETURN_HOME

        self.return_release_arm_q = self._get_hold_arm_angles()
        self.state = State.RETURN_RELEASE
        self.state_timer = 0.0
        self.get_logger().warn(f'{reason}，开始执行回原始安全位流程（先张开夹爪，再撤离）。')

    def execute_safe_shutdown(self) -> None:
        if not self.return_home_on_shutdown:
            self.get_logger().info('已关闭退出回原位功能，控制节点将直接退出。')
            return
        if not rclpy.ok():
            self.get_logger().warn(
                'ROS 上下文已不可用，自动回原位无法继续发布命令；请优先使用手动回原位指令。'
            )
            return
        if not self._ensure_named_waypoints():
            self.get_logger().warn('退出控制流程失败：存在不可达的安全位，已跳过自动回原位。')
            return

        try:
            self.timer.cancel()
        except Exception:
            pass

        hold_arm_angles = self._get_hold_arm_angles()
        shutdown_sequence = []
        shutdown_sequence.append(('原地张开夹爪', hold_arm_angles, self.gripper_open_angle))

        if self.state in (State.GRASP, State.PINCH, State.SHARED_CONTROL):
            shutdown_sequence.append(('抬升到悬停位', self.q_hover, self.gripper_open_angle))
            shutdown_sequence.append(('回撤到过渡位', self.q_transit, self.gripper_open_angle))
        elif self.state == State.HOVER:
            shutdown_sequence.append(('回撤到过渡位', self.q_transit, self.gripper_open_angle))
        if self.state not in (State.HOME, State.TRANSIT):
            shutdown_sequence.append(('回到 Home 并保持夹爪张开', self.q_home, self.gripper_open_angle))

        self.get_logger().warn('检测到控制节点中断，开始执行自动回原始安全位流程（先张开夹爪，再撤离）。')
        for segment_name, target_q, target_gripper_angle in shutdown_sequence:
            self._run_shutdown_segment(target_q, target_gripper_angle, segment_name)
        self.get_logger().info('自动回原始安全位流程完成。')

    def control_loop(self) -> None:
        self.state_timer += self.dt
        err_limit = 0.065

        if self.state != State.SHARED_CONTROL:
            self.publish_shared_feedback((0.0, 0.0), 0.0, False)

        if self.return_home_requested and self.state != State.INIT:
            self._start_return_home_sequence('收到手动回原位请求')

        if self.state == State.INIT:
            self.q_home = self.kinematics.inverse_kinematics(self.home_xyz, self.target_phi)
            self.q_transit = self.kinematics.inverse_kinematics(self.transit_xyz, self.target_phi)
            self.q_hover = self.kinematics.inverse_kinematics(self.hover_xyz, self.target_phi)
            self.q_grasp = self.kinematics.inverse_kinematics(self.grasp_xyz, self.target_phi)

            if any(q is None for q in [self.q_home, self.q_transit, self.q_hover, self.q_grasp]):
                self.get_logger().error('致命错误：预设轨迹点存在不可达位置，控制器保持在 INIT。')
                self.state = State.INIT
            else:
                self.state = State.HOME
                self.state_timer = 0.0
            return

        if self.state == State.HOME:
            self.smooth_and_publish(self.q_home, self.gripper_open_angle)
            if np.linalg.norm(self.current_q[:4] - self.q_home) < err_limit and self.state_timer > 2.0:
                self.state = State.TRANSIT
                self.state_timer = 0.0
            return

        if self.state == State.SAFE_HOLD:
            self.smooth_and_publish(self.q_home, self.gripper_open_angle)
            return

        if self.state == State.TRANSIT:
            self.smooth_and_publish(self.q_transit, self.gripper_open_angle)
            if np.linalg.norm(self.current_q[:4] - self.q_transit) < err_limit and self.state_timer > 2.0:
                self.state = State.HOVER
                self.state_timer = 0.0
            return

        if self.state == State.HOVER:
            self.smooth_and_publish(self.q_hover, self.gripper_open_angle)
            if np.linalg.norm(self.current_q[:4] - self.q_hover) < err_limit and self.state_timer > 2.0:
                self.state = State.GRASP
                self.state_timer = 0.0
            return

        if self.state == State.GRASP:
            self.smooth_and_publish(self.q_grasp, self.gripper_open_angle)
            if np.linalg.norm(self.current_q[:4] - self.q_grasp) < err_limit and self.state_timer > 2.0:
                self.get_logger().info('到达抓取点，准备闭合夹爪。')
                self.shared_gripper_hold_angle = self.current_gripper_cmd
                self.pinch_contact_start_time = None
                self.contact_detected_gripper_angle = None
                self.state = State.PINCH
                self.state_timer = 0.0
            return

        if self.state == State.PINCH:
            target_gripper_angle = self.current_gripper_cmd
            if self.gripper_sensor_fault:
                self.pinch_contact_start_time = None
                target_gripper_angle = max(
                    self.current_gripper_cmd,
                    self.gripper_relief_angle,
                )
                self.get_logger().warn(
                    'PINCH 阶段检测到 FSR 传感异常，暂停夹持确认并轻微放松夹爪。',
                    throttle_duration_sec=1.0,
                )
            elif self.gripper_overpressure:
                self.pinch_contact_start_time = None
                target_gripper_angle = max(
                    self.current_gripper_cmd,
                    self.gripper_relief_angle,
                )
                self.get_logger().warn(
                    f'PINCH 阶段检测到夹持过压，夹爪轻微回退保护。'
                    f' CurrentAngle={self.current_gripper_cmd:.3f}'
                    f' → ReliefAngle={self.gripper_relief_angle:.3f}',
                    throttle_duration_sec=1.0,
                )
            elif self.gripper_contact_required and self.gripper_contact_ok:
                if self.pinch_contact_start_time is None:
                    self.pinch_contact_start_time = self.state_timer
                    self.contact_detected_gripper_angle = self._clamp_gripper_angle(
                        self.current_gripper_cmd
                    )
                    (
                        self.shared_gripper_hold_angle,
                        hold_mode,
                        hold_note,
                    ) = self._select_grip_hold_angle(self.contact_detected_gripper_angle)
                    self.get_logger().info(
                        (
                            '检测到夹持接触，开始稳定确认: '
                            f'ContactAngle={self.contact_detected_gripper_angle:.3f} | '
                            f'HoldAngle={self.shared_gripper_hold_angle:.3f} | '
                            f'HoldMode={hold_mode}{hold_note}'
                        )
                    )
                target_gripper_angle = self.shared_gripper_hold_angle
            elif self.gripper_contact_required:
                self.pinch_contact_start_time = None
                self.contact_detected_gripper_angle = None
                target_gripper_angle = self._clamp_gripper_angle(
                    self.current_gripper_cmd - self.pinch_close_rate_per_s * self.dt
                )
            else:
                self.pinch_contact_start_time = None
                self.contact_detected_gripper_angle = None
                if self.use_fixed_grip_hold_angle:
                    target_gripper_angle = self.fixed_grip_hold_angle
                else:
                    target_gripper_angle = self._clamp_gripper_angle(
                        self.current_gripper_cmd - self.pinch_close_rate_per_s * self.dt
                    )

            self.smooth_and_publish(self.q_grasp, target_gripper_angle)
            if self.state_timer % 1.0 < self.dt:
                if self.gripper_contact_required:
                    self.get_logger().info('夹爪闭合中，等待 FSR 接触确认...')
                else:
                    self.get_logger().info(
                        '夹爪闭合中，当前为共享控制调参路径：按固定角/时间进入共享控制。'
                    )

            if self.gripper_contact_required:
                if (
                    self.pinch_contact_start_time is not None
                    and (self.state_timer - self.pinch_contact_start_time) > self.pinch_contact_hold_s
                ):
                    self.get_logger().info(
                        (
                            '夹取确认（FSR 已检测到稳定接触），进入共享控制模式。'
                            f' ContactAngle={self.contact_detected_gripper_angle:.3f}'
                            f' | HoldAngle={self.shared_gripper_hold_angle:.3f}'
                        )
                    )
                    self.active_xyz = self.grasp_xyz.copy()
                    self.shared_offset_xy = np.zeros(2, dtype=float)
                    self.shared_contact_loss_start_time = None
                    self.regrasp_attempt_count = 0
                    self.shared_sub_state = 1
                    self.state = State.SHARED_CONTROL
                    self.state_timer = 0.0
                return

            if self.state_timer > 2.0:
                if self.use_fixed_grip_hold_angle:
                    self.shared_gripper_hold_angle = self.fixed_grip_hold_angle
                else:
                    self.shared_gripper_hold_angle = self._clamp_gripper_angle(
                        self.current_gripper_cmd
                    )
                self.get_logger().info(
                    f'抓取确认，进入共享控制模式。持握角度={self.shared_gripper_hold_angle:.3f}'
                )
                self.active_xyz = self.grasp_xyz.copy()
                self.shared_offset_xy = np.zeros(2, dtype=float)
                self.shared_contact_loss_start_time = None
                self.shared_sub_state = 1
                self.state = State.SHARED_CONTROL
                self.state_timer = 0.0
            return

        if self.state == State.SHARED_CONTROL:
            target_gripper_angle = self.shared_gripper_hold_angle
            desired_offset_xy = np.zeros(2, dtype=float)
            virtual_correction_xy = np.zeros(2, dtype=float)
            laser_recenter_xy = np.zeros(2, dtype=float)
            safety_target_xy = np.zeros(2, dtype=float)
            feedback_xy_norm = np.zeros(2, dtype=float)
            authority_alpha = 0.0
            hard_recenter_active = False
            free_zone_active = self.laser_radius_px <= self.laser_safe_radius_px + 1e-6

            if (
                self.shared_control_bypass_fsr
                and (
                    self.gripper_sensor_fault
                    or self.gripper_overpressure
                    or (self.gripper_contact_required and not self.gripper_contact_ok)
                )
            ):
                self.get_logger().warn(
                    (
                        '共享控制调参模式：检测到 FSR 状态变化，但已旁路保护。'
                        f' Contact={int(self.gripper_contact_ok)} | '
                        f'Over={int(self.gripper_overpressure)} | '
                        f'Fault={int(self.gripper_sensor_fault)}'
                    ),
                    throttle_duration_sec=1.0,
                )

            if (not self.shared_control_bypass_fsr) and self.gripper_sensor_fault:
                self.shared_contact_loss_start_time = None
                target_gripper_angle = max(
                    self.shared_gripper_hold_angle,
                    self.gripper_relief_angle,
                )
                candidate_offset_xy = np.zeros(2, dtype=float)
                self.net_force = np.zeros(3, dtype=float)
                self.get_logger().warn(
                    '共享控制阶段检测到 FSR 传感异常，机械臂退回中性抓取位并轻微放松夹爪。',
                    throttle_duration_sec=1.0,
                )
            elif (not self.shared_control_bypass_fsr) and self.gripper_overpressure:
                self.shared_contact_loss_start_time = None
                self.regrasp_attempt_count += 1
                if self.regrasp_attempt_count > self.regrasp_max_attempts:
                    self._start_return_home_sequence(
                        f'共享控制过压且重抓次数已达上限（{self.regrasp_max_attempts}次）'
                    )
                else:
                    self.get_logger().warn(
                        f'共享控制阶段检测到夹持过压，进入缓冲释放后重新夹取'
                        f'（第{self.regrasp_attempt_count}/{self.regrasp_max_attempts}次）。'
                    )
                    self.state = State.REGRASP
                    self.state_timer = 0.0
                return
            elif (
                (not self.shared_control_bypass_fsr)
                and self.gripper_contact_required
                and not self.gripper_contact_ok
            ):
                if self.shared_contact_loss_start_time is None:
                    self.shared_contact_loss_start_time = self.state_timer

                contact_loss_elapsed = (
                    self.state_timer - self.shared_contact_loss_start_time
                )
                candidate_offset_xy = self.shared_offset_xy.copy()
                self.net_force = np.zeros(3, dtype=float)
                if contact_loss_elapsed <= self.shared_contact_loss_timeout_s:
                    self.get_logger().warn(
                        (
                            '共享控制阶段夹持接触短暂丢失，保持当前姿态等待恢复。'
                            f' Loss={contact_loss_elapsed:.2f}s / '
                            f'{self.shared_contact_loss_timeout_s:.2f}s'
                        ),
                        throttle_duration_sec=0.5,
                    )
                else:
                    if self.shared_control_lock_state:
                        self.get_logger().warn(
                            f'共享控制阶段接触丢失超时（{self.shared_contact_loss_timeout_s:.1f}s），'
                            '状态锁定模式：保持姿态继续等待，不退出共享控制。',
                            throttle_duration_sec=2.0,
                        )
                        self.shared_contact_loss_start_time = self.state_timer
                    else:
                        self.get_logger().warn(
                            f'共享控制阶段接触丢失超时（{self.shared_contact_loss_timeout_s:.1f}s），'
                            '重新进入 PINCH 状态重新夹取。'
                        )
                        self.shared_contact_loss_start_time = None
                        self.pinch_contact_start_time = None
                        self.contact_detected_gripper_angle = None
                        self.state = State.PINCH
                        self.state_timer = 0.0
                        self.smooth_and_publish(self.q_grasp, self.shared_gripper_hold_angle)
                        return
            else:
                if self.shared_contact_loss_start_time is not None:
                    recovered_dt = self.state_timer - self.shared_contact_loss_start_time
                    self.get_logger().info(
                        f'共享控制阶段夹持接触已恢复，接触丢失持续 {recovered_dt:.2f}s。'
                    )
                self.shared_contact_loss_start_time = None
                (
                    desired_offset_xy,
                    virtual_correction_xy,
                    laser_recenter_xy,
                    safety_target_xy,
                    candidate_offset_xy,
                    authority_alpha,
                    hard_recenter_active,
                ) = (
                    self._compute_target_offset_xy()
                )
                feedback_xy_norm = np.divide(
                    safety_target_xy,
                    self.max_joystick_offset_xy,
                    out=np.zeros(2, dtype=float),
                    where=self.max_joystick_offset_xy > 1e-9,
                )
                feedback_xy_norm = np.clip(feedback_xy_norm, -1.0, 1.0)
                if hard_recenter_active and self.shared_sub_state != 2:
                    self.shared_sub_state = 2
                    self.get_logger().info(
                        '激光点超出安全范围，切换到机器完全主导：持续大幅反向回推摇杆。'
                        f' DistNorm={self.laser_dist_norm:.2f}'
                    )
                elif (not hard_recenter_active) and self.shared_sub_state != 1:
                    self.shared_sub_state = 1
                    self.get_logger().info(
                        '激光点已回到安全范围内，切换回范围内共享控制（alpha + virtual force）。'
                        f' DistNorm={self.laser_dist_norm:.2f}'
                    )
                if self.shared_sub_state == 1:
                    if free_zone_active:
                        target_gripper_angle = self._clamp_gripper_angle(
                            self.shared_gripper_hold_angle + self.shared_idle_open_extra_m
                        )
                    else:
                        target_gripper_angle = self.shared_gripper_hold_angle
                else:
                    target_gripper_angle = self.shared_gripper_hold_angle
                self.net_force = np.array(
                    [self.virtual_force[0], self.virtual_force[1], 0.0],
                    dtype=float,
                )
            self.authority_alpha = authority_alpha
            self.publish_shared_feedback(
                feedback_xy_norm,
                authority_alpha,
                hard_recenter_active,
            )

            candidate_xyz = self.grasp_xyz.copy()
            candidate_xyz[0] += candidate_offset_xy[0]
            candidate_xyz[1] += candidate_offset_xy[1]
            candidate_xyz[2] = self.shared_control_height_m

            q_active = self.kinematics.inverse_kinematics(candidate_xyz, self.target_phi)
            if q_active is not None:
                self.shared_offset_xy = candidate_offset_xy
                self.active_xyz = candidate_xyz
                self.smooth_and_publish(q_active, target_gripper_angle)
            else:
                self.get_logger().warn(
                    '共享控制局部偏移超出可达空间，机械臂保持上一合法姿态。',
                    throttle_duration_sec=1.0,
                )

            if self.state_timer % 0.5 < self.dt:
                self.get_logger().info(
                    (
                        f'共享控制 | Joy={np.round(self.joy_xy, 3)} | '
                        f'Virtual={np.round(self.virtual_force[:2], 3)} | '
                        f'Alpha={self.authority_alpha:.2f} | '
                        f'CtrlMode={"hard_return" if hard_recenter_active else "shared_blend"} | '
                        f'SubState={self.shared_sub_state} | '
                        f'GripZone={"free_open" if free_zone_active and self.shared_sub_state == 1 else "hold"} | '
                        f'DistNorm={self.laser_dist_norm:.2f} | '
                        f'LaserPx={self.laser_radius_px:.1f} | '
                        f'Free/Bound={self.laser_safe_radius_px:.1f}/{self.laser_boundary_radius_px:.1f} | '
                        f'VirtualCorr={np.round(virtual_correction_xy, 4)} | '
                        f'LaserRecent={np.round(laser_recenter_xy, 4)} | '
                        f'SafetyTarget={np.round(safety_target_xy, 4)} | '
                        f'FbNorm={np.round(feedback_xy_norm, 3)} | '
                        f'Offset={np.round(self.shared_offset_xy, 4)} | '
                        f'Contact={int(self.gripper_contact_ok)} | '
                        f'Over={int(self.gripper_overpressure)} | '
                        f'Fault={int(self.gripper_sensor_fault)} | '
                        f'ContactAngle={"未记录" if self.contact_detected_gripper_angle is None else ("全闭时触" if self.use_fixed_grip_hold_angle and self.contact_detected_gripper_angle < 0.005 else f"{self.contact_detected_gripper_angle:.3f}")} | '
                        f'GripCmd={target_gripper_angle:.3f} | '
                        f'XYZ={np.round(self.active_xyz, 4)}'
                    )
                )
            return

        if self.state == State.RETURN_RELEASE:
            target_q = (
                self.return_release_arm_q
                if self.return_release_arm_q is not None
                else self._get_hold_arm_angles()
            )
            self.smooth_and_publish(target_q, self.gripper_open_angle)
            if self.state_timer > self.shutdown_step_duration_s:
                next_state = self.return_after_release_state
                if next_state == State.RETURN_HOVER:
                    self.get_logger().info('回原位流程：夹爪已张开，准备抬升到悬停位。')
                elif next_state == State.RETURN_TRANSIT:
                    self.get_logger().info('回原位流程：夹爪已张开，准备回撤到过渡位。')
                else:
                    self.get_logger().info('回原位流程：夹爪已张开，准备回 Home。')
                self.state = next_state
                self.state_timer = 0.0
            return

        if self.state == State.RETURN_HOVER:
            self.smooth_and_publish(self.q_hover, self.gripper_open_angle)
            if (
                np.linalg.norm(self.current_q[:4] - self.q_hover) < err_limit
                and self.state_timer > self.shutdown_step_duration_s
            ):
                self.get_logger().info('回原位流程：已到达悬停位，准备回撤到过渡位。')
                self.state = State.RETURN_TRANSIT
                self.state_timer = 0.0
            return

        if self.state == State.RETURN_TRANSIT:
            self.smooth_and_publish(self.q_transit, self.gripper_open_angle)
            if (
                np.linalg.norm(self.current_q[:4] - self.q_transit) < err_limit
                and self.state_timer > self.shutdown_step_duration_s
            ):
                self.get_logger().info('回原位流程：已到达过渡位，准备回 Home。')
                self.state = State.RETURN_HOME
                self.state_timer = 0.0
            return

        if self.state == State.RETURN_HOME:
            self.smooth_and_publish(self.q_home, self.gripper_open_angle)
            if np.linalg.norm(self.current_q[:4] - self.q_home) < err_limit and self.state_timer > self.shutdown_step_duration_s:
                self.get_logger().info('回原位流程完成：机械臂已返回安全 Home 位。')
                self.state = State.SAFE_HOLD
                self.state_timer = 0.0
            return

        if self.state == State.REGRASP:
            self.smooth_and_publish(self.q_grasp, self.gripper_relief_angle)
            if self.state_timer < self.regrasp_relief_duration_s:
                return
            if self.gripper_overpressure:
                if self.state_timer > self.regrasp_relief_duration_s + 2.0:
                    self._start_return_home_sequence('REGRASP 阶段等待过压消除超时（2s）')
                return
            self.get_logger().info(
                f'REGRASP 阶段过压已消除，重新进入 PINCH 重新夹取'
                f'（第{self.regrasp_attempt_count}/{self.regrasp_max_attempts}次）。'
            )
            self.pinch_contact_start_time = None
            self.contact_detected_gripper_angle = None
            self.state = State.PINCH
            self.state_timer = 0.0
            return

    def smooth_and_publish(self, target_arm_angles, target_gripper_angle: float) -> None:
        if self.smooth_q is None:
            if np.any(self.current_q):
                self.smooth_q = np.copy(self.current_q[:4])
            else:
                self.smooth_q = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

        self.smooth_q = (1.0 - self.alpha) * self.smooth_q + self.alpha * np.array(
            target_arm_angles,
            dtype=float,
        )
        clamped_gripper_angle = self._clamp_gripper_angle(target_gripper_angle)
        self.current_gripper_cmd = clamped_gripper_angle
        self.publish_cmd(self.smooth_q, clamped_gripper_angle)

    def publish_cmd(self, arm_angles, gripper_angle: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(angle) for angle in arm_angles] + [float(gripper_angle)]
        self.cmd_pub.publish(msg)

    def publish_shared_feedback(
        self,
        feedback_xy_norm: np.ndarray | list[float] | tuple[float, float],
        authority_alpha: float = 0.0,
        hard_recenter_active: bool = False,
    ) -> None:
        feedback_xy = np.array(feedback_xy_norm[:2], dtype=float)
        feedback_xy = np.clip(feedback_xy, -1.0, 1.0)
        msg = Float32MultiArray()
        msg.data = [
            float(feedback_xy[0]),
            float(feedback_xy[1]),
            float(max(0.0, min(1.0, authority_alpha))),
            1.0 if hard_recenter_active else 0.0,
        ]
        self.shared_feedback_pub.publish(msg)


def main(args=None):
    init_kwargs = {}
    if SignalHandlerOptions is not None and hasattr(SignalHandlerOptions, 'NO'):
        init_kwargs['signal_handler_options'] = SignalHandlerOptions.NO
    try:
        rclpy.init(args=args, **init_kwargs)
    except TypeError:
        rclpy.init(args=args)
    node = ArmControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.execute_safe_shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
