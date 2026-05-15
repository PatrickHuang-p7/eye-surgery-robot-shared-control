import os
import math
import threading
import time
from collections import deque
from typing import Tuple

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    import pygame
except ImportError:  # pragma: no cover - 运行环境缺库时走降级路径
    pygame = None


class PygameHapticSimNode(Node):
    """在 Pygame 中绘制虚拟夹具，并把惩罚阻力发布回机械臂控制节点。"""

    def __init__(self):
        super().__init__('pygame_haptic_sim_node')

        self.declare_parameter('screen_width', 960)
        self.declare_parameter('screen_height', 720)
        self.declare_parameter('render_fps', 60.0)
        self.declare_parameter('safe_zone_radius_px', 220.0)
        self.declare_parameter('buffer_width_px', 60.0)
        self.declare_parameter('max_force_n', 8.0)
        self.declare_parameter('laser_speed_px_per_s', 260.0)
        self.declare_parameter('laser_input_exponent', 1.6)
        self.declare_parameter('laser_max_speed_px_per_s', 200.0)
        self.declare_parameter('laser_max_accel_px_per_s2', 800.0)
        self.declare_parameter('laser_limit_radius_px', 320.0)
        self.declare_parameter('shared_feedback_scale', 1.0)
        self.declare_parameter('trail_enabled', True)
        self.declare_parameter('trail_max_points', 1500)
        self.declare_parameter('trail_line_width', 3)
        self.declare_parameter('trail_min_point_distance_px', 1.5)
        self.declare_parameter('enable_capture_hotkey', False)
        self.declare_parameter('capture_output_dir', '/tmp/pygame_haptic_sim_captures')

        self.screen_width = int(self.get_parameter('screen_width').value)
        self.screen_height = int(self.get_parameter('screen_height').value)
        self.render_fps = max(1.0, float(self.get_parameter('render_fps').value))
        self.safe_zone_radius_px = max(1.0, float(self.get_parameter('safe_zone_radius_px').value))
        self.buffer_width_px = max(0.0, float(self.get_parameter('buffer_width_px').value))
        self.max_force_n = max(0.0, float(self.get_parameter('max_force_n').value))
        self.laser_speed_px_per_s = max(
            1.0,
            float(self.get_parameter('laser_speed_px_per_s').value),
        )
        self.laser_input_exponent = max(
            1.0,
            float(self.get_parameter('laser_input_exponent').value),
        )
        self.laser_max_speed_px_per_s = max(
            1.0,
            float(self.get_parameter('laser_max_speed_px_per_s').value),
        )
        self.laser_max_accel_px_per_s2 = max(
            1.0,
            float(self.get_parameter('laser_max_accel_px_per_s2').value),
        )
        self.laser_limit_radius_px = max(
            self.safe_zone_radius_px,
            float(self.get_parameter('laser_limit_radius_px').value),
        )
        self.shared_feedback_scale = max(
            0.0,
            float(self.get_parameter('shared_feedback_scale').value),
        )
        self.trail_enabled = bool(self.get_parameter('trail_enabled').value)
        self.trail_max_points = max(10, int(self.get_parameter('trail_max_points').value))
        self.trail_line_width = max(1, int(self.get_parameter('trail_line_width').value))
        self.trail_min_point_distance_px = max(
            0.0,
            float(self.get_parameter('trail_min_point_distance_px').value),
        )
        self.enable_capture_hotkey = bool(
            self.get_parameter('enable_capture_hotkey').value
        )
        self.capture_output_dir = str(self.get_parameter('capture_output_dir').value).strip()

        self.force_pub = self.create_publisher(Float32MultiArray, '/virtual_fixture_force', 10)
        self.laser_pos_pub = self.create_publisher(Float32MultiArray, '/laser_position', 10)
        self.create_subscription(Float32MultiArray, '/joy_position', self.joy_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            '/shared_control_feedback',
            self.shared_feedback_callback,
            10,
        )

        self._lock = threading.Lock()
        self._latest_joy = [0.0, 0.0, 0.0, 0.0]
        self._latest_shared_feedback = [0.0, 0.0, 0.0, 0.0]
        self._effective_joy_xy = [0.0, 0.0]
        self._laser_xy_px = [0.0, 0.0]
        self._laser_velocity_px_s = [0.0, 0.0]
        self._trail_points = deque(maxlen=self.trail_max_points)
        self._stop_event = threading.Event()

        self.get_logger().info('Pygame 虚拟夹具节点已启动，等待 /joy_position 数据...')

    def joy_callback(self, msg: Float32MultiArray) -> None:
        with self._lock:
            data = list(msg.data[:4])
            while len(data) < 4:
                data.append(0.0)
            self._latest_joy = [float(v) for v in data]

    def get_latest_joy(self) -> list[float]:
        with self._lock:
            return list(self._latest_joy)

    def shared_feedback_callback(self, msg: Float32MultiArray) -> None:
        with self._lock:
            data = list(msg.data[:4])
            while len(data) < 4:
                data.append(0.0)
            self._latest_shared_feedback = [float(v) for v in data]

    def get_latest_shared_feedback(self) -> list[float]:
        with self._lock:
            return list(self._latest_shared_feedback)

    def publish_virtual_force(self, fx: float, fy: float) -> None:
        if not rclpy.ok():
            return
        msg = Float32MultiArray()
        msg.data = [float(fx), float(fy), 0.0]
        try:
            self.force_pub.publish(msg)
        except Exception as exc:  # pragma: no cover - ROS 关闭阶段保护
            self.get_logger().warn(
                f'虚拟力发布失败，已忽略本次收尾发布: {exc}',
                throttle_duration_sec=1.0,
            )

    def _publish_laser_position(self, x_px: float, y_px: float, radius: float) -> None:
        if not rclpy.ok():
            return
        norm_denom = max(self.safe_zone_radius_px, 1e-9)
        msg = Float32MultiArray()
        free_radius = max(0.0, self.safe_zone_radius_px - self.buffer_width_px)
        msg.data = [
            float(x_px / norm_denom),
            float(y_px / norm_denom),
            float(radius / norm_denom),
            float(free_radius / norm_denom),
            float(radius),
            float(free_radius),
            float(self.safe_zone_radius_px),
        ]
        try:
            self.laser_pos_pub.publish(msg)
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(
                f'激光位置发布失败: {exc}',
                throttle_duration_sec=1.0,
            )

    def request_stop(self) -> None:
        self._stop_event.set()

    def clear_trail(self) -> None:
        self._trail_points.clear()

    def _append_trail_point(
        self,
        x_px: float,
        y_px: float,
        radius: float,
        free_radius: float,
        hard_return_active: bool,
    ) -> None:
        if not self.trail_enabled:
            return

        if self._trail_points:
            last_x, last_y, _, _, _ = self._trail_points[-1]
            if math.hypot(x_px - last_x, y_px - last_y) < self.trail_min_point_distance_px:
                self._trail_points[-1] = (
                    x_px,
                    y_px,
                    radius,
                    free_radius,
                    hard_return_active,
                )
                return

        self._trail_points.append((x_px, y_px, radius, free_radius, hard_return_active))

    def _trail_segment_color(
        self,
        radius: float,
        free_radius: float,
        hard_return_active: bool,
    ) -> tuple[int, int, int]:
        if hard_return_active or radius > self.safe_zone_radius_px:
            return (255, 76, 76)
        if radius > free_radius:
            return (255, 226, 43)
        return (26, 229, 255)

    def _shape_human_input(self, value: float) -> float:
        clipped = max(-1.0, min(1.0, value))
        magnitude = abs(clipped)
        shaped = magnitude ** self.laser_input_exponent
        return math.copysign(shaped, clipped)

    def _draw_trail(self, screen: pygame.Surface, center_x: int, center_y: int) -> None:
        if not self.trail_enabled or len(self._trail_points) < 2:
            return

        trail_surface = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
        points = list(self._trail_points)
        point_count = len(points)
        for idx in range(1, point_count):
            prev_x, prev_y, _, _, _ = points[idx - 1]
            curr_x, curr_y, curr_radius, curr_free_radius, curr_hard = points[idx]
            progress = idx / max(1, point_count - 1)
            alpha = int(10 + 235 * (progress ** 1.8))
            color = self._trail_segment_color(curr_radius, curr_free_radius, curr_hard)
            pygame.draw.line(
                trail_surface,
                (*color, alpha),
                (int(center_x + prev_x), int(center_y - prev_y)),
                (int(center_x + curr_x), int(center_y - curr_y)),
                self.trail_line_width,
            )

        screen.blit(trail_surface, (0, 0))

    def _draw_trail_markers(self, screen: pygame.Surface, center_x: int, center_y: int) -> None:
        if not self._trail_points:
            return

        start_x, start_y, _, _, _ = self._trail_points[0]
        start_screen = (int(center_x + start_x), int(center_y - start_y))
        pygame.draw.circle(screen, (255, 255, 255), start_screen, 8, width=2)
        pygame.draw.circle(screen, (26, 229, 255), start_screen, 4)

    def _draw_info_panel(
        self,
        screen: pygame.Surface,
        font: pygame.font.Font,
        lines: list[str],
        anchor: str,
        margin_x: int,
        margin_y: int,
        fill_color: tuple[int, int, int, int],
        border_color: tuple[int, int, int],
        text_color: tuple[int, int, int],
    ) -> None:
        if not lines:
            return

        horizontal_padding = 16
        vertical_padding = 12
        line_height = 28
        panel_width = max(font.size(line)[0] for line in lines) + horizontal_padding * 2
        panel_height = vertical_padding * 2 + len(lines) * line_height - 6

        x_pos = margin_x
        y_pos = margin_y
        if 'right' in anchor:
            x_pos = self.screen_width - margin_x - panel_width
        if 'bottom' in anchor:
            y_pos = self.screen_height - margin_y - panel_height

        panel_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        pygame.draw.rect(
            panel_surface,
            fill_color,
            pygame.Rect(0, 0, panel_width, panel_height),
            border_radius=14,
        )
        pygame.draw.rect(
            panel_surface,
            border_color,
            pygame.Rect(0, 0, panel_width, panel_height),
            width=1,
            border_radius=14,
        )
        screen.blit(panel_surface, (x_pos, y_pos))

        for idx, text in enumerate(lines):
            surface = font.render(text, True, text_color)
            screen.blit(
                surface,
                (
                    x_pos + horizontal_padding,
                    y_pos + vertical_padding + idx * line_height,
                ),
            )

    def save_capture(self, screen: pygame.Surface) -> str:
        output_dir = self.capture_output_dir or '/tmp/pygame_haptic_sim_captures'
        os.makedirs(output_dir, exist_ok=True)
        filename = f'laser_trail_{time.strftime("%Y%m%d_%H%M%S")}.png'
        output_path = os.path.join(output_dir, filename)
        pygame.image.save(screen, output_path)
        return output_path

    def _compute_effective_joy(
        self,
        joy_x: float,
        joy_y: float,
        feedback_x: float,
        feedback_y: float,
    ) -> Tuple[float, float]:
        shaped_joy_x = self._shape_human_input(joy_x)
        shaped_joy_y = self._shape_human_input(joy_y)
        effective_x = max(
            -1.0,
            min(1.0, shaped_joy_x + self.shared_feedback_scale * feedback_x),
        )
        effective_y = max(
            -1.0,
            min(1.0, shaped_joy_y + self.shared_feedback_scale * feedback_y),
        )
        self._effective_joy_xy[0] = effective_x
        self._effective_joy_xy[1] = effective_y
        return effective_x, effective_y

    @staticmethod
    def _clip_vector_magnitude(x_val: float, y_val: float, max_norm: float) -> Tuple[float, float]:
        norm = math.hypot(x_val, y_val)
        if norm <= max_norm or norm <= 1e-9:
            return x_val, y_val
        scale = max_norm / norm
        return x_val * scale, y_val * scale

    def _integrate_laser(
        self,
        effective_joy_x: float,
        effective_joy_y: float,
        dt: float,
    ) -> Tuple[float, float, float, float]:
        target_vx = effective_joy_x * self.laser_speed_px_per_s
        target_vy = effective_joy_y * self.laser_speed_px_per_s
        target_vx, target_vy = self._clip_vector_magnitude(
            target_vx,
            target_vy,
            self.laser_max_speed_px_per_s,
        )

        curr_vx = self._laser_velocity_px_s[0]
        curr_vy = self._laser_velocity_px_s[1]
        delta_vx = target_vx - curr_vx
        delta_vy = target_vy - curr_vy
        delta_speed = math.hypot(delta_vx, delta_vy)
        max_delta_speed = self.laser_max_accel_px_per_s2 * dt
        if delta_speed > max_delta_speed and delta_speed > 1e-9:
            scale = max_delta_speed / delta_speed
            delta_vx *= scale
            delta_vy *= scale

        curr_vx += delta_vx
        curr_vy += delta_vy
        curr_vx, curr_vy = self._clip_vector_magnitude(
            curr_vx,
            curr_vy,
            self.laser_max_speed_px_per_s,
        )

        self._laser_velocity_px_s[0] = curr_vx
        self._laser_velocity_px_s[1] = curr_vy
        self._laser_xy_px[0] += curr_vx * dt
        self._laser_xy_px[1] += curr_vy * dt

        radius = math.hypot(self._laser_xy_px[0], self._laser_xy_px[1])
        if radius > self.laser_limit_radius_px:
            scale = self.laser_limit_radius_px / max(radius, 1e-9)
            self._laser_xy_px[0] *= scale
            self._laser_xy_px[1] *= scale
            unit_x = self._laser_xy_px[0] / max(self.laser_limit_radius_px, 1e-9)
            unit_y = self._laser_xy_px[1] / max(self.laser_limit_radius_px, 1e-9)
            outward_speed = curr_vx * unit_x + curr_vy * unit_y
            if outward_speed > 0.0:
                curr_vx -= outward_speed * unit_x
                curr_vy -= outward_speed * unit_y
                self._laser_velocity_px_s[0] = curr_vx
                self._laser_velocity_px_s[1] = curr_vy

        return self._laser_xy_px[0], self._laser_xy_px[1], curr_vx, curr_vy

    def compute_virtual_force(self, laser_xy_px: Tuple[float, float]) -> Tuple[float, float, float, float]:
        """基于人工势场法计算阻力，坐标系采用数学坐标系（Y 向上）。"""
        x_px, y_px = laser_xy_px
        radius = math.hypot(x_px, y_px)
        free_radius = max(0.0, self.safe_zone_radius_px - self.buffer_width_px)

        if radius <= 1e-9:
            return 0.0, 0.0, radius, free_radius

        if radius <= free_radius:
            magnitude = 0.0
        elif radius <= self.safe_zone_radius_px and self.buffer_width_px > 1e-9:
            magnitude = self.max_force_n * (radius - free_radius) / self.buffer_width_px
        else:
            magnitude = self.max_force_n

        unit_x = x_px / radius
        unit_y = y_px / radius
        fx = -magnitude * unit_x
        fy = -magnitude * unit_y
        return fx, fy, radius, free_radius

    def run_pygame_loop(self) -> None:
        if pygame is None:
            raise RuntimeError('未安装 pygame，无法启动虚拟夹具窗口。')

        pygame.init()
        screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption('CPS Haptic Safe Zone')
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('monospace', 20)

        center_x = self.screen_width // 2
        center_y = self.screen_height // 2
        background_color = (7, 10, 18)
        crosshair_color = (72, 86, 112)
        text_color = (239, 244, 255)
        panel_fill_color = (8, 12, 22, 210)
        panel_border_color = (110, 124, 152)
        free_fill_color = (24, 96, 132, 150)
        buffer_fill_color = (255, 188, 0, 52)
        free_boundary_color = (26, 229, 255)
        safe_boundary_color = (255, 226, 43)
        limit_boundary_color = (255, 92, 158)
        laser_outer_color = (255, 255, 255)
        laser_core_color = (255, 92, 158)
        dt = 1.0 / self.render_fps

        try:
            while not self._stop_event.is_set() and rclpy.ok():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.request_stop()
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        self.request_stop()
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                        self.clear_trail()
                        self.get_logger().info('已清空激光轨迹，可开始下一次实验。')
                    elif (
                        self.enable_capture_hotkey
                        and event.type == pygame.KEYDOWN
                        and event.key == pygame.K_s
                    ):
                        output_path = self.save_capture(screen)
                        self.get_logger().info(f'已保存当前轨迹截图: {output_path}')

                joy_x, joy_y, joy_vx, joy_vy = self.get_latest_joy()
                feedback_x, feedback_y, feedback_alpha, feedback_hard = (
                    self.get_latest_shared_feedback()
                )
                effective_joy_x, effective_joy_y = self._compute_effective_joy(
                    joy_x,
                    joy_y,
                    feedback_x,
                    feedback_y,
                )
                laser_x_px, laser_y_px, laser_vx_px_s, laser_vy_px_s = self._integrate_laser(
                    effective_joy_x,
                    effective_joy_y,
                    dt,
                )

                force_x, force_y, radius, free_radius = self.compute_virtual_force(
                    (laser_x_px, laser_y_px)
                )
                self._append_trail_point(
                    laser_x_px,
                    laser_y_px,
                    radius,
                    free_radius,
                    feedback_hard > 0.5,
                )
                self.publish_virtual_force(force_x, force_y)
                self._publish_laser_position(laser_x_px, laser_y_px, radius)

                screen.fill(background_color)
                pygame.draw.line(
                    screen,
                    crosshair_color,
                    (0, center_y),
                    (self.screen_width, center_y),
                    1,
                )
                pygame.draw.line(
                    screen,
                    crosshair_color,
                    (center_x, 0),
                    (center_x, self.screen_height),
                    1,
                )

                overlay_surface = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)
                pygame.draw.circle(
                    overlay_surface,
                    buffer_fill_color,
                    (center_x, center_y),
                    int(self.safe_zone_radius_px),
                )
                pygame.draw.circle(
                    overlay_surface,
                    free_fill_color,
                    (center_x, center_y),
                    int(max(0.0, free_radius)),
                )
                screen.blit(overlay_surface, (0, 0))
                pygame.draw.circle(
                    screen,
                    free_boundary_color,
                    (center_x, center_y),
                    int(max(0.0, free_radius)),
                    width=3,
                )
                pygame.draw.circle(
                    screen,
                    safe_boundary_color,
                    (center_x, center_y),
                    int(self.safe_zone_radius_px),
                    width=4,
                )
                pygame.draw.circle(
                    screen,
                    limit_boundary_color,
                    (center_x, center_y),
                    int(self.laser_limit_radius_px),
                    width=2,
                )
                self._draw_trail(screen, center_x, center_y)
                self._draw_trail_markers(screen, center_x, center_y)

                laser_screen_x = int(center_x + laser_x_px)
                laser_screen_y = int(center_y - laser_y_px)
                pygame.draw.circle(screen, laser_outer_color, (laser_screen_x, laser_screen_y), 12)
                pygame.draw.circle(screen, laser_core_color, (laser_screen_x, laser_screen_y), 7)
                pygame.draw.circle(screen, laser_outer_color, (laser_screen_x, laser_screen_y), 22, width=2)

                info_lines = [
                    f'Joy Input : x={joy_x:+.2f} y={joy_y:+.2f}',
                    f'Joy Eff  : x={effective_joy_x:+.2f} y={effective_joy_y:+.2f}',
                    f'Joy Vel  : vx={joy_vx:+.2f} vy={joy_vy:+.2f}',
                    f'Feedback : x={feedback_x:+.2f} y={feedback_y:+.2f} a={feedback_alpha:.2f}',
                    f'Laser    : x={laser_x_px:+.1f}px y={laser_y_px:+.1f}px',
                    f'Laser Vel: vx={laser_vx_px_s:+.1f} vy={laser_vy_px_s:+.1f} px/s',
                    f'Distance : {radius:.1f}px',
                    f'Margin   : {(self.safe_zone_radius_px - radius):+.1f}px',
                    f'Force    : Fx={force_x:+.2f}N Fy={force_y:+.2f}N',
                    f'Expo     : human^{self.laser_input_exponent:.2f}',
                    f'Limit    : v<={self.laser_max_speed_px_per_s:.0f}px/s a<={self.laser_max_accel_px_per_s2:.0f}px/s²',
                    (
                        f'Trail    : {len(self._trail_points)} pts | C=clear S=save'
                        if self.enable_capture_hotkey
                        else f'Trail    : {len(self._trail_points)} pts | C=clear'
                    ),
                ]
                top_left_lines = info_lines[0:3]
                top_right_lines = info_lines[3:6]
                bottom_left_lines = info_lines[6:9]
                bottom_right_lines = info_lines[9:11]

                self._draw_info_panel(
                    screen,
                    font,
                    top_left_lines,
                    'topleft',
                    20,
                    16,
                    panel_fill_color,
                    panel_border_color,
                    text_color,
                )
                self._draw_info_panel(
                    screen,
                    font,
                    top_right_lines,
                    'topright',
                    20,
                    16,
                    panel_fill_color,
                    panel_border_color,
                    text_color,
                )
                self._draw_info_panel(
                    screen,
                    font,
                    bottom_left_lines,
                    'bottomleft',
                    20,
                    16,
                    panel_fill_color,
                    panel_border_color,
                    text_color,
                )
                self._draw_info_panel(
                    screen,
                    font,
                    bottom_right_lines,
                    'bottomright',
                    20,
                    16,
                    panel_fill_color,
                    panel_border_color,
                    text_color,
                )

                pygame.display.flip()
                dt = max(1e-3, clock.tick(self.render_fps) / 1000.0)

                self.get_logger().info(
                    (
                        f'虚拟夹具 | Joy=({joy_x:+.2f}, {joy_y:+.2f}) '
                        f'Fb=({feedback_x:+.2f}, {feedback_y:+.2f}) '
                        f'Eff=({effective_joy_x:+.2f}, {effective_joy_y:+.2f}) '
                        f'Laser=({laser_x_px:+.1f}px, {laser_y_px:+.1f}px) '
                        f'Force=({force_x:+.2f}, {force_y:+.2f}) '
                        f'{"[hard_return]" if feedback_hard > 0.5 else ""}'
                    ),
                    throttle_duration_sec=1.0,
                )
        finally:
            if rclpy.ok():
                self.publish_virtual_force(0.0, 0.0)
            pygame.quit()


def _spin_executor(executor: SingleThreadedExecutor) -> None:
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PygameHapticSimNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=_spin_executor, args=(executor,), daemon=True)
    spin_thread.start()

    try:
        node.run_pygame_loop()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    finally:
        node.request_stop()
        if rclpy.ok():
            node.publish_virtual_force(0.0, 0.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
