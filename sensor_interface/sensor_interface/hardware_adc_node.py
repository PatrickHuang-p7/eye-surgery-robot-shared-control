import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

try:
    import smbus2
except ImportError:  # pragma: no cover - 运行环境缺库时走降级路径
    smbus2 = None


class HardwareADCNode(Node):
    """从 ADS1115 读取 FSR 与摇杆模拟量，并转换为 ROS 2 控制消息。"""

    REG_CONVERSION = 0x00
    REG_CONFIG = 0x01
    ADS_FSR = 4.096
    ADS_RESOLUTION = 32768.0
    V_IN = 3.3
    R_PULLUP_KOHM = 10.0

    def __init__(self):
        super().__init__('hardware_adc_node')

        # ---------------------------
        # ADS1115 与标定参数
        # ---------------------------
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('ads_address', 0x48)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('joystick_only_mode', False)
        self.declare_parameter('enable_grip_sensor_fault_detection', False)

        self.declare_parameter('force_zero_x', 0.0)
        self.declare_parameter('force_zero_y', 0.0)
        self.declare_parameter('force_gain_x', 0.00981)
        self.declare_parameter('force_gain_y', 0.00981)
        self.declare_parameter('force_sign_x', 1.0)
        self.declare_parameter('force_sign_y', 1.0)
        self.declare_parameter('force_voltage_jump_threshold', 0.40)
        self.declare_parameter('force_confirm_samples', 2)
        self.declare_parameter('force_ema_alpha', 0.35)
        self.declare_parameter('force_max_n', 20.0)
        self.declare_parameter('grip_contact_threshold_left_n', 11.0)
        self.declare_parameter('grip_contact_threshold_right_n', 8.5)
        self.declare_parameter('grip_contact_release_left_n', 7.0)
        self.declare_parameter('grip_contact_release_right_n', 5.5)
        self.declare_parameter('grip_overpressure_threshold_left_n', 19.4)
        self.declare_parameter('grip_overpressure_threshold_right_n', 15.2)
        self.declare_parameter('grip_require_both_contact', True)
        self.declare_parameter('use_left_fsr_for_contact', True)
        self.declare_parameter('use_right_fsr_for_contact', True)
        self.declare_parameter('use_left_fsr_for_overpressure', True)
        self.declare_parameter('use_right_fsr_for_overpressure', True)
        self.declare_parameter('grip_fault_low_voltage_v', 0.03)
        self.declare_parameter('grip_fault_force_saturation_n', 19.6)
        self.declare_parameter('grip_fault_confirm_samples', 3)
        self.declare_parameter('grip_fault_recover_samples', 5)

        self.declare_parameter('joy_center_x', 1.65)
        self.declare_parameter('joy_center_y', 1.65)
        self.declare_parameter('joy_span_x', 1.65)
        self.declare_parameter('joy_span_y', 1.65)
        self.declare_parameter('joy_deadband', 0.05)
        self.declare_parameter('invert_joy_x', False)
        self.declare_parameter('invert_joy_y', False)
        self.declare_parameter('joy_voltage_jump_threshold', 0.35)
        self.declare_parameter('joy_confirm_samples', 2)
        self.declare_parameter('joy_ema_alpha', 0.30)

        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.ads_address = int(self.get_parameter('ads_address').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.joystick_only_mode = bool(self.get_parameter('joystick_only_mode').value)
        self.enable_grip_sensor_fault_detection = bool(
            self.get_parameter('enable_grip_sensor_fault_detection').value
        )

        self.force_zero = [
            float(self.get_parameter('force_zero_x').value),
            float(self.get_parameter('force_zero_y').value),
        ]
        self.force_gain = [
            float(self.get_parameter('force_gain_x').value),
            float(self.get_parameter('force_gain_y').value),
        ]
        self.force_sign = [
            float(self.get_parameter('force_sign_x').value),
            float(self.get_parameter('force_sign_y').value),
        ]
        self.force_voltage_jump_threshold = max(
            0.0,
            float(self.get_parameter('force_voltage_jump_threshold').value),
        )
        self.force_confirm_samples = max(
            1,
            int(self.get_parameter('force_confirm_samples').value),
        )
        self.force_ema_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter('force_ema_alpha').value)),
        )
        self.force_max_n = max(0.0, float(self.get_parameter('force_max_n').value))
        self.grip_contact_threshold = [
            max(0.0, float(self.get_parameter('grip_contact_threshold_left_n').value)),
            max(0.0, float(self.get_parameter('grip_contact_threshold_right_n').value)),
        ]
        self.grip_contact_release_threshold = [
            max(0.0, float(self.get_parameter('grip_contact_release_left_n').value)),
            max(0.0, float(self.get_parameter('grip_contact_release_right_n').value)),
        ]
        self.grip_contact_release_threshold = [
            min(self.grip_contact_threshold[0], self.grip_contact_release_threshold[0]),
            min(self.grip_contact_threshold[1], self.grip_contact_release_threshold[1]),
        ]
        self.grip_overpressure_threshold = [
            max(0.0, float(self.get_parameter('grip_overpressure_threshold_left_n').value)),
            max(0.0, float(self.get_parameter('grip_overpressure_threshold_right_n').value)),
        ]
        self.grip_require_both_contact = bool(
            self.get_parameter('grip_require_both_contact').value
        )
        self.use_left_fsr_for_contact = bool(
            self.get_parameter('use_left_fsr_for_contact').value
        )
        self.use_right_fsr_for_contact = bool(
            self.get_parameter('use_right_fsr_for_contact').value
        )
        self.use_left_fsr_for_overpressure = bool(
            self.get_parameter('use_left_fsr_for_overpressure').value
        )
        self.use_right_fsr_for_overpressure = bool(
            self.get_parameter('use_right_fsr_for_overpressure').value
        )
        self.grip_fault_low_voltage_v = max(
            0.0,
            float(self.get_parameter('grip_fault_low_voltage_v').value),
        )
        self.grip_fault_force_saturation_n = max(
            0.0,
            float(self.get_parameter('grip_fault_force_saturation_n').value),
        )
        self.grip_fault_confirm_samples = max(
            1,
            int(self.get_parameter('grip_fault_confirm_samples').value),
        )
        self.grip_fault_recover_samples = max(
            1,
            int(self.get_parameter('grip_fault_recover_samples').value),
        )

        self.joy_center = [
            float(self.get_parameter('joy_center_x').value),
            float(self.get_parameter('joy_center_y').value),
        ]
        self.joy_span = [
            max(1e-6, float(self.get_parameter('joy_span_x').value)),
            max(1e-6, float(self.get_parameter('joy_span_y').value)),
        ]
        self.joy_deadband = max(0.0, float(self.get_parameter('joy_deadband').value))
        self.invert_joy_x = bool(self.get_parameter('invert_joy_x').value)
        self.invert_joy_y = bool(self.get_parameter('invert_joy_y').value)
        self.joy_voltage_jump_threshold = max(
            0.0,
            float(self.get_parameter('joy_voltage_jump_threshold').value),
        )
        self.joy_confirm_samples = max(
            1,
            int(self.get_parameter('joy_confirm_samples').value),
        )
        self.joy_ema_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter('joy_ema_alpha').value)),
        )

        # ---------------------------
        # ROS 2 接口
        # ---------------------------
        self.force_pub = self.create_publisher(Float32MultiArray, '/force_sensor', 10)
        self.joy_pub = self.create_publisher(Float32MultiArray, '/joy_position', 10)
        self.gripper_contact_pub = self.create_publisher(Bool, '/gripper_contact', 10)
        self.gripper_overpressure_pub = self.create_publisher(Bool, '/gripper_overpressure', 10)
        self.gripper_sensor_fault_pub = self.create_publisher(Bool, '/gripper_sensor_fault', 10)

        self.bus = None
        self.is_ready = False
        self.last_retry_time = 0.0
        self.prev_joy = [0.0, 0.0]
        self.prev_sample_time = None
        self.signal_filters = self._create_signal_filters()
        self.grip_fault_state = self._create_grip_fault_state()
        self.grip_contact_state = self._create_grip_contact_state()

        self._initialize_bus()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.read_and_publish)
        self.get_logger().info(
            '硬件 ADC 节点已启动: A0->GripL, A1->GripR, A2->JoyX, A3->JoyY'
        )
        if self.enable_grip_sensor_fault_detection:
            self.get_logger().info('FSR 故障检测已开启：将监测掉线/浮空/异常饱和。')
        else:
            self.get_logger().warn('FSR 故障检测当前已关闭：夹持判定将直接使用 FSR 力值结果。')
        self.get_logger().info(
            'FSR 判定通道: '
            f'Contact={self._describe_enabled_channels(self.use_left_fsr_for_contact, self.use_right_fsr_for_contact)} | '
            f'Overpressure={self._describe_enabled_channels(self.use_left_fsr_for_overpressure, self.use_right_fsr_for_overpressure)}'
        )
        if not (self.use_left_fsr_for_contact or self.use_right_fsr_for_contact):
            self.get_logger().warn('接触判定已禁用左右两侧 FSR，/gripper_contact 将始终为 False。')
        if not (
            self.use_left_fsr_for_overpressure or self.use_right_fsr_for_overpressure
        ):
            self.get_logger().warn(
                '过压判定已禁用左右两侧 FSR，/gripper_overpressure 将始终为 False。'
            )

    def _initialize_bus(self) -> None:
        """尝试建立 I2C 连接；失败时保活节点并等待下次重连。"""
        self.last_retry_time = time.monotonic()

        if smbus2 is None:
            self.is_ready = False
            self.get_logger().error(
                '未检测到 smbus2 Python 库，hardware_adc_node 将持续发布零值。'
            )
            return

        try:
            if self.bus is not None:
                self.bus.close()
            self.bus = smbus2.SMBus(self.i2c_bus)
            self.bus.read_byte(self.ads_address)
            self.is_ready = True
            self.get_logger().info(
                f'已连接 ADS1115: bus={self.i2c_bus}, address=0x{self.ads_address:02X}'
            )
        except Exception as exc:  # pragma: no cover - 依赖真实硬件
            self.bus = None
            self.is_ready = False
            self.get_logger().error(
                f'ADS1115 初始化失败: {exc}。节点将保活并周期重试。'
            )

    def _create_signal_filters(self) -> dict:
        return {
            'force_x': self._new_filter_state(),
            'force_y': self._new_filter_state(),
            'joy_x': self._new_filter_state(),
            'joy_y': self._new_filter_state(),
        }

    def _new_filter_state(self) -> dict:
        return {
            'stable': None,
            'candidate': None,
            'candidate_count': 0,
        }

    def _reset_filters(self) -> None:
        self.signal_filters = self._create_signal_filters()
        self.grip_fault_state = self._create_grip_fault_state()
        self.grip_contact_state = self._create_grip_contact_state()

    def _create_grip_fault_state(self) -> dict:
        return {
            'left': self._new_grip_fault_channel_state(),
            'right': self._new_grip_fault_channel_state(),
        }

    def _create_grip_contact_state(self) -> dict:
        return {
            'left': False,
            'right': False,
        }

    def _describe_enabled_channels(self, use_left: bool, use_right: bool) -> str:
        enabled = []
        if use_left:
            enabled.append('left')
        if use_right:
            enabled.append('right')
        if not enabled:
            return 'none'
        return '+'.join(enabled)

    def _new_grip_fault_channel_state(self) -> dict:
        return {
            'fault': False,
            'bad_count': 0,
            'good_count': 0,
        }

    def _maybe_retry_initialize(self) -> None:
        if self.is_ready:
            return
        if time.monotonic() - self.last_retry_time >= 1.0:
            self._initialize_bus()

    def _read_adc_voltage(self, channel: int) -> float:
        """使用单次转换 + OS 位轮询，确保每次读到当前通道的新鲜数据。"""
        if not self.is_ready or self.bus is None:
            raise RuntimeError('ADS1115 未就绪')

        config = 0x0000
        config |= (1 << 15)             # OS: 启动一次单次转换
        config |= ((4 + channel) << 12) # MUX: AINx 相对 GND
        config |= (1 << 9)              # PGA: ±4.096V
        config |= (1 << 8)              # MODE: 单次转换
        config |= (4 << 5)              # DR: 128 SPS
        config |= 0x0003                # 禁用比较器

        config_h = (config >> 8) & 0xFF
        config_l = config & 0xFF
        self.bus.write_i2c_block_data(self.ads_address, self.REG_CONFIG, [config_h, config_l])

        while True:
            status = self.bus.read_i2c_block_data(self.ads_address, self.REG_CONFIG, 2)
            if status[0] & 0x80:
                break
            time.sleep(0.001)

        result = self.bus.read_i2c_block_data(self.ads_address, self.REG_CONVERSION, 2)
        raw_adc = (result[0] << 8) | result[1]
        if raw_adc & 0x8000:
            raw_adc -= 0x10000

        return raw_adc * (self.ADS_FSR / self.ADS_RESOLUTION)

    def _sanitize_voltage(self, voltage: float) -> float:
        return max(0.0, min(self.V_IN, voltage))

    def _calculate_fsr_force_scalar(self, voltage: float) -> float:
        """复用现有 FSR 标定曲线，先得到原始受力标量（单位与标定参数一致）。"""
        if voltage > 3.1:
            return 0.0

        voltage = max(0.001, min(voltage, self.V_IN - 0.001))
        r_fsr = self.R_PULLUP_KOHM * (voltage / (self.V_IN - voltage))
        r_fsr = max(r_fsr, 0.1)
        force_grams = (194.46 / r_fsr) ** (1.0 / 0.665)
        return force_grams

    def _normalize_axis(self, raw_voltage: float, center: float, span: float, invert: bool) -> float:
        normalized = max(-1.0, min(1.0, (raw_voltage - center) / span))
        if invert:
            normalized *= -1.0
        if abs(normalized) < self.joy_deadband:
            normalized = 0.0
        return normalized

    def _filter_signal(
        self,
        key: str,
        raw_voltage: float,
        jump_threshold: float,
        confirm_samples: int,
        ema_alpha: float,
    ) -> tuple[float, bool]:
        """
        为每一路模拟量加一道“确认制”护栏：
        单帧大跳变先保持上一稳定值，只有连续多帧都落在同一新区域才接受。
        """
        state = self.signal_filters[key]
        raw_voltage = self._sanitize_voltage(raw_voltage)

        if state['stable'] is None:
            state['stable'] = raw_voltage
            state['candidate'] = None
            state['candidate_count'] = 0
            return raw_voltage, False

        stable_voltage = state['stable']
        if abs(raw_voltage - stable_voltage) <= jump_threshold:
            filtered_voltage = (
                ema_alpha * raw_voltage + (1.0 - ema_alpha) * stable_voltage
            )
            state['stable'] = filtered_voltage
            state['candidate'] = None
            state['candidate_count'] = 0
            return filtered_voltage, False

        candidate_voltage = state['candidate']
        candidate_match_threshold = max(jump_threshold * 2.0, 0.12)
        if (
            candidate_voltage is None
            or abs(raw_voltage - candidate_voltage) > candidate_match_threshold
        ):
            state['candidate'] = raw_voltage
            state['candidate_count'] = 1
            return stable_voltage, True

        state['candidate_count'] += 1
        if state['candidate_count'] < confirm_samples:
            return stable_voltage, True

        filtered_voltage = (
            ema_alpha * raw_voltage + (1.0 - ema_alpha) * stable_voltage
        )
        state['stable'] = filtered_voltage
        state['candidate'] = None
        state['candidate_count'] = 0
        return filtered_voltage, False

    def _compute_velocity(self, x_now: float, y_now: float) -> tuple[float, float]:
        now = time.monotonic()
        if self.prev_sample_time is None:
            self.prev_sample_time = now
            self.prev_joy = [x_now, y_now]
            return 0.0, 0.0

        dt = max(1e-6, now - self.prev_sample_time)
        vx = max(-1.0, min(1.0, (x_now - self.prev_joy[0]) / dt))
        vy = max(-1.0, min(1.0, (y_now - self.prev_joy[1]) / dt))

        self.prev_sample_time = now
        self.prev_joy = [x_now, y_now]
        return vx, vy

    def _publish_force(self, fx: float, fy: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(fx), float(fy), 0.0]
        self.force_pub.publish(msg)

    def _publish_joy(self, x_norm: float, y_norm: float, vx: float, vy: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(x_norm), float(y_norm), float(vx), float(vy)]
        self.joy_pub.publish(msg)

    def _publish_grip_status(self, contact_ok: bool, overpressure: bool) -> None:
        contact_msg = Bool()
        contact_msg.data = bool(contact_ok)
        self.gripper_contact_pub.publish(contact_msg)

        overpressure_msg = Bool()
        overpressure_msg.data = bool(overpressure)
        self.gripper_overpressure_pub.publish(overpressure_msg)

    def _publish_grip_sensor_fault(self, sensor_fault: bool) -> None:
        msg = Bool()
        msg.data = bool(sensor_fault)
        self.gripper_sensor_fault_pub.publish(msg)

    def _publish_zero_outputs(self) -> None:
        self.prev_joy = [0.0, 0.0]
        self.prev_sample_time = None
        self._reset_filters()
        self._publish_force(0.0, 0.0)
        self._publish_joy(0.0, 0.0, 0.0, 0.0)
        self._publish_grip_status(False, False)
        self._publish_grip_sensor_fault(False)

    def _convert_force_axis(self, voltage: float, axis_index: int) -> float:
        raw_force = self._calculate_fsr_force_scalar(voltage)
        calibrated_force = max(0.0, raw_force - self.force_zero[axis_index])
        force_newtons = (
            self.force_sign[axis_index]
            * self.force_gain[axis_index]
            * calibrated_force
        )
        return max(-self.force_max_n, min(self.force_max_n, force_newtons))

    def _evaluate_grip_state(
        self,
        left_force_n: float,
        right_force_n: float,
    ) -> tuple[bool, bool, bool, bool]:
        left_contact = self._update_grip_contact_channel(
            'left',
            left_force_n,
            self.grip_contact_threshold[0],
            self.grip_contact_release_threshold[0],
        )
        right_contact = self._update_grip_contact_channel(
            'right',
            right_force_n,
            self.grip_contact_threshold[1],
            self.grip_contact_release_threshold[1],
        )

        enabled_contact_channels = []
        if self.use_left_fsr_for_contact:
            enabled_contact_channels.append(left_contact)
        if self.use_right_fsr_for_contact:
            enabled_contact_channels.append(right_contact)

        if not enabled_contact_channels:
            contact_ok = False
        elif self.grip_require_both_contact:
            contact_ok = all(enabled_contact_channels)
        else:
            contact_ok = any(enabled_contact_channels)

        enabled_overpressure_channels = []
        if self.use_left_fsr_for_overpressure:
            enabled_overpressure_channels.append(
                left_force_n >= self.grip_overpressure_threshold[0]
            )
        if self.use_right_fsr_for_overpressure:
            enabled_overpressure_channels.append(
                right_force_n >= self.grip_overpressure_threshold[1]
            )
        overpressure = any(enabled_overpressure_channels)
        return left_contact, right_contact, contact_ok, overpressure

    def _update_grip_contact_channel(
        self,
        channel_name: str,
        force_n: float,
        engage_threshold: float,
        release_threshold: float,
    ) -> bool:
        engaged = bool(self.grip_contact_state[channel_name])
        if engaged:
            engaged = force_n >= release_threshold
        else:
            engaged = force_n >= engage_threshold
        self.grip_contact_state[channel_name] = engaged
        return engaged

    def _update_grip_fault_channel(
        self,
        channel_name: str,
        raw_voltage: float,
        force_n: float,
    ) -> bool:
        state = self.grip_fault_state[channel_name]
        suspicious = (
            raw_voltage <= self.grip_fault_low_voltage_v
            or force_n >= self.grip_fault_force_saturation_n
        )

        if suspicious:
            state['bad_count'] += 1
            state['good_count'] = 0
            if state['bad_count'] >= self.grip_fault_confirm_samples:
                state['fault'] = True
        else:
            state['good_count'] += 1
            state['bad_count'] = 0
            if state['good_count'] >= self.grip_fault_recover_samples:
                state['fault'] = False

        return bool(state['fault'])

    def read_and_publish(self) -> None:
        """读取 4 路 ADC，转换为力觉与摇杆状态，并发布给控制链路。"""
        if not self.is_ready:
            self._maybe_retry_initialize()
            if not self.is_ready:
                self.get_logger().warn(
                    'ADS1115 尚未就绪，本周期发布零值。',
                    throttle_duration_sec=5.0,
                )
                self._publish_zero_outputs()
                return

        try:
            rejected_signals = []

            if self.joystick_only_mode:
                v_fx = self.V_IN
                v_fy = self.V_IN
                fx = 0.0
                fy = 0.0
                raw_v_fx = self.V_IN
                raw_v_fy = self.V_IN
                left_contact = False
                right_contact = False
                grip_contact_ok = False
                grip_overpressure = False
                left_fault = False
                right_fault = False
                grip_sensor_fault = False
            else:
                raw_v_fx = self._read_adc_voltage(0)
                raw_v_fy = self._read_adc_voltage(1)
                v_fx, rejected_fx = self._filter_signal(
                    'force_x',
                    raw_v_fx,
                    self.force_voltage_jump_threshold,
                    self.force_confirm_samples,
                    self.force_ema_alpha,
                )
                v_fy, rejected_fy = self._filter_signal(
                    'force_y',
                    raw_v_fy,
                    self.force_voltage_jump_threshold,
                    self.force_confirm_samples,
                    self.force_ema_alpha,
                )
                if rejected_fx:
                    rejected_signals.append('Fx')
                if rejected_fy:
                    rejected_signals.append('Fy')
                fx = self._convert_force_axis(v_fx, 0)
                fy = self._convert_force_axis(v_fy, 1)
                if self.enable_grip_sensor_fault_detection:
                    left_fault = self._update_grip_fault_channel('left', raw_v_fx, fx)
                    right_fault = self._update_grip_fault_channel('right', raw_v_fy, fy)
                    grip_sensor_fault = left_fault or right_fault
                else:
                    left_fault = False
                    right_fault = False
                    grip_sensor_fault = False

                if grip_sensor_fault:
                    fx = 0.0
                    fy = 0.0
                    left_contact = False
                    right_contact = False
                    grip_contact_ok = False
                    grip_overpressure = False
                else:
                    (
                        left_contact,
                        right_contact,
                        grip_contact_ok,
                        grip_overpressure,
                    ) = self._evaluate_grip_state(fx, fy)

            raw_v_joy_x = self._read_adc_voltage(2)
            raw_v_joy_y = self._read_adc_voltage(3)
            v_joy_x, rejected_joy_x = self._filter_signal(
                'joy_x',
                raw_v_joy_x,
                self.joy_voltage_jump_threshold,
                self.joy_confirm_samples,
                self.joy_ema_alpha,
            )
            v_joy_y, rejected_joy_y = self._filter_signal(
                'joy_y',
                raw_v_joy_y,
                self.joy_voltage_jump_threshold,
                self.joy_confirm_samples,
                self.joy_ema_alpha,
            )
            if rejected_joy_x:
                rejected_signals.append('JoyX')
            if rejected_joy_y:
                rejected_signals.append('JoyY')

            x_norm = self._normalize_axis(
                v_joy_x,
                self.joy_center[0],
                self.joy_span[0],
                self.invert_joy_x,
            )
            y_norm = self._normalize_axis(
                v_joy_y,
                self.joy_center[1],
                self.joy_span[1],
                self.invert_joy_y,
            )
            vx, vy = self._compute_velocity(x_norm, y_norm)

            self._publish_force(fx, fy)
            self._publish_joy(x_norm, y_norm, vx, vy)
            self._publish_grip_status(grip_contact_ok, grip_overpressure)
            self._publish_grip_sensor_fault(grip_sensor_fault)

            if rejected_signals:
                self.get_logger().warn(
                    f'检测到可疑跳变，已保持上一稳定值: {", ".join(rejected_signals)}',
                    throttle_duration_sec=1.0,
                )

            if grip_sensor_fault:
                fault_axes = []
                if left_fault:
                    fault_axes.append('GripL')
                if right_fault:
                    fault_axes.append('GripR')
                self.get_logger().warn(
                    (
                        '检测到 FSR 传感异常，已忽略本周期夹持力判定: '
                        f'{", ".join(fault_axes)}'
                    ),
                    throttle_duration_sec=1.0,
                )

            self.get_logger().info(
                (
                    f'ADC采样 | GripRawV=(A0:{raw_v_fx:.3f}V, A1:{raw_v_fy:.3f}V) | '
                    f'Grip(L={fx:.2f}N, R={fy:.2f}N) '
                    f'Contact=({int(left_contact)},{int(right_contact)})->{int(grip_contact_ok)} '
                    f'Over={int(grip_overpressure)} Fault={int(grip_sensor_fault)} | '
                    f'JoyRawV=(A2:{raw_v_joy_x:.3f}V, A3:{raw_v_joy_y:.3f}V) | '
                    f'Joy=({x_norm:.2f}, {y_norm:.2f}) Vel=({vx:.2f}, {vy:.2f})'
                ),
                throttle_duration_sec=1.0,
            )
        except Exception as exc:  # pragma: no cover - 依赖真实硬件
            self.is_ready = False
            self.get_logger().error(
                f'ADS1115 读取失败: {exc}。本周期输出已清零，并将在后台自动重连。',
                throttle_duration_sec=1.0,
            )
            self._publish_zero_outputs()

    def destroy_node(self) -> bool:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareADCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
