import argparse
import math

try:
    import pygame
except ImportError:  # pragma: no cover - 本地未安装 pygame 时给出友好提示
    pygame = None

try:
    from .shared_control_math import SharedControlParams
    from .shared_control_math import compute_target_offset
    from .shared_control_math import compute_virtual_force
    from .shared_control_math import integrate_laser
except ImportError:  # pragma: no cover - 兼容直接 python 执行文件
    from shared_control_math import SharedControlParams
    from shared_control_math import compute_target_offset
    from shared_control_math import compute_virtual_force
    from shared_control_math import integrate_laser


class KeyboardPreviewApp:
    def __init__(self, args: argparse.Namespace):
        self.screen_width = args.screen_width
        self.screen_height = args.screen_height
        self.render_fps = args.render_fps
        self.params = SharedControlParams(
            safe_zone_radius_px=args.safe_zone_radius_px,
            buffer_width_px=args.buffer_width_px,
            max_force_n=args.max_force_n,
            laser_speed_px_per_s=args.laser_speed_px_per_s,
            laser_limit_radius_px=args.laser_limit_radius_px,
            max_offset_x_m=args.max_offset_x_m,
            max_offset_y_m=args.max_offset_y_m,
            follow_scale_x_m=args.follow_scale_x_m,
            follow_scale_y_m=args.follow_scale_y_m,
            virtual_force_gain_x=args.virtual_force_gain_x,
            virtual_force_gain_y=args.virtual_force_gain_y,
            authority_force_start_n=args.authority_force_start_n,
            authority_force_full_n=args.authority_force_full_n,
            laser_recenter_gain=args.laser_recenter_gain,
        )
        self.laser_xy_px = (0.0, 0.0)

    @property
    def free_radius_px(self) -> float:
        return self.params.free_radius_px

    def run(self) -> None:
        if pygame is None:
            raise RuntimeError('未安装 pygame。先执行: python3 -m pip install pygame')

        pygame.init()
        screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption('Shared Control Keyboard Preview')
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('monospace', 20)

        center_x = self.screen_width // 2
        center_y = self.screen_height // 2
        bg = (14, 22, 31)
        safe_ring = (56, 88, 116)
        safe_fill = (70, 142, 118)
        border = (217, 120, 78)
        laser_color = (255, 82, 82)
        command_color = (105, 214, 255)
        text_color = (236, 241, 245)

        dt = 1.0 / self.render_fps
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.laser_xy_px = (0.0, 0.0)

            keys = pygame.key.get_pressed()
            joy_x = 0.0
            joy_y = 0.0
            if keys[pygame.K_LEFT]:
                joy_x -= 1.0
            if keys[pygame.K_RIGHT]:
                joy_x += 1.0
            if keys[pygame.K_DOWN]:
                joy_y -= 1.0
            if keys[pygame.K_UP]:
                joy_y += 1.0
            if joy_x and joy_y:
                norm = math.hypot(joy_x, joy_y)
                joy_x /= norm
                joy_y /= norm

            self.laser_xy_px = integrate_laser(self.laser_xy_px, joy_x, joy_y, dt, self.params)
            laser_x_px, laser_y_px = self.laser_xy_px
            force_x, force_y, radius_px = compute_virtual_force(laser_x_px, laser_y_px, self.params)
            offset_data = compute_target_offset(
                joy_x,
                joy_y,
                laser_x_px,
                laser_y_px,
                force_x,
                force_y,
                radius_px,
                self.params,
            )

            screen.fill(bg)
            pygame.draw.circle(screen, safe_ring, (center_x, center_y), int(self.params.safe_zone_radius_px))
            pygame.draw.circle(screen, safe_fill, (center_x, center_y), int(self.free_radius_px))
            pygame.draw.circle(
                screen,
                border,
                (center_x, center_y),
                int(self.params.safe_zone_radius_px),
                width=4,
            )

            laser_screen_x = int(center_x + laser_x_px)
            laser_screen_y = int(center_y - laser_y_px)
            pygame.draw.circle(screen, laser_color, (laser_screen_x, laser_screen_y), 10)
            pygame.draw.circle(screen, (255, 244, 167), (laser_screen_x, laser_screen_y), 20, width=2)

            offset_x, offset_y = offset_data.target_offset_xy
            offset_marker_x = int(
                center_x + (offset_x / max(self.params.max_offset_x_m, 1e-9)) * self.free_radius_px
            )
            offset_marker_y = int(
                center_y - (offset_y / max(self.params.max_offset_y_m, 1e-9)) * self.free_radius_px
            )
            pygame.draw.circle(screen, command_color, (offset_marker_x, offset_marker_y), 8)
            pygame.draw.line(
                screen,
                command_color,
                (center_x, center_y),
                (offset_marker_x, offset_marker_y),
                width=3,
            )

            info_lines = [
                'Arrow keys: joystick input | Space: recenter laser | Esc: quit',
                f'Joy            x={joy_x:+.2f} y={joy_y:+.2f}',
                f'LaserPx        r={radius_px:6.1f} free={self.free_radius_px:6.1f} bound={self.params.safe_zone_radius_px:6.1f}',
                f'Force          Fx={force_x:+.2f} Fy={force_y:+.2f} | |F|={offset_data.force_norm:.2f}',
                f'Alpha          {offset_data.alpha:.2f}',
                f'CtrlMode       {"hard_return" if offset_data.hard_recenter_active else "shared_blend"}',
                f'LaserRecent    x={offset_data.laser_recenter_xy[0]:+.4f} y={offset_data.laser_recenter_xy[1]:+.4f}',
                f'SafetyTarget   x={offset_data.safety_target_xy[0]:+.4f} y={offset_data.safety_target_xy[1]:+.4f}',
                f'TargetOffset   x={offset_x:+.4f} y={offset_y:+.4f}',
            ]

            for idx, line in enumerate(info_lines):
                surface = font.render(line, True, text_color)
                screen.blit(surface, (24, 24 + idx * 28))

            pygame.display.flip()
            dt = max(1e-3, clock.tick(self.render_fps) / 1000.0)

        pygame.quit()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Keyboard preview for shared-control logic.')
    parser.add_argument('--screen-width', type=int, default=960)
    parser.add_argument('--screen-height', type=int, default=720)
    parser.add_argument('--render-fps', type=float, default=60.0)
    parser.add_argument('--safe-zone-radius-px', type=float, default=220.0)
    parser.add_argument('--buffer-width-px', type=float, default=60.0)
    parser.add_argument('--max-force-n', type=float, default=8.0)
    parser.add_argument('--laser-speed-px-per-s', type=float, default=260.0)
    parser.add_argument('--laser-limit-radius-px', type=float, default=320.0)
    parser.add_argument('--max-offset-x-m', type=float, default=0.008)
    parser.add_argument('--max-offset-y-m', type=float, default=0.008)
    parser.add_argument('--follow-scale-x-m', type=float, default=0.009)
    parser.add_argument('--follow-scale-y-m', type=float, default=0.009)
    parser.add_argument('--virtual-force-gain-x', type=float, default=0.0009)
    parser.add_argument('--virtual-force-gain-y', type=float, default=0.0009)
    parser.add_argument('--authority-force-start-n', type=float, default=1.0)
    parser.add_argument('--authority-force-full-n', type=float, default=4.0)
    parser.add_argument('--laser-recenter-gain', type=float, default=0.85)
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    app = KeyboardPreviewApp(args)
    app.run()


if __name__ == '__main__':
    main()
