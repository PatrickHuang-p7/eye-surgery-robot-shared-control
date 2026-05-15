from __future__ import annotations

from dataclasses import dataclass
import math


def clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class SharedControlParams:
    safe_zone_radius_px: float = 220.0
    buffer_width_px: float = 60.0
    max_force_n: float = 8.0
    laser_speed_px_per_s: float = 260.0
    laser_limit_radius_px: float = 320.0
    max_offset_x_m: float = 0.008
    max_offset_y_m: float = 0.008
    follow_scale_x_m: float = 0.009
    follow_scale_y_m: float = 0.009
    virtual_force_gain_x: float = 0.0009
    virtual_force_gain_y: float = 0.0009
    authority_force_start_n: float = 1.0
    authority_force_full_n: float = 4.0
    laser_recenter_gain: float = 0.85

    @property
    def free_radius_px(self) -> float:
        return max(0.0, self.safe_zone_radius_px - self.buffer_width_px)

    @property
    def authority_full_clamped(self) -> float:
        return max(self.authority_force_full_n, self.authority_force_start_n + 1e-6)


@dataclass(frozen=True)
class OffsetComputation:
    desired_offset_xy: tuple[float, float]
    virtual_correction_xy: tuple[float, float]
    laser_recenter_xy: tuple[float, float]
    safety_target_xy: tuple[float, float]
    target_offset_xy: tuple[float, float]
    alpha: float
    force_norm: float
    hard_recenter_active: bool


def integrate_laser(
    laser_xy_px: tuple[float, float],
    joy_x: float,
    joy_y: float,
    dt: float,
    params: SharedControlParams,
) -> tuple[float, float]:
    laser_x = laser_xy_px[0] + joy_x * params.laser_speed_px_per_s * dt
    laser_y = laser_xy_px[1] + joy_y * params.laser_speed_px_per_s * dt

    radius = math.hypot(laser_x, laser_y)
    if radius > params.laser_limit_radius_px:
        scale = params.laser_limit_radius_px / max(radius, 1e-9)
        laser_x *= scale
        laser_y *= scale

    return laser_x, laser_y


def compute_virtual_force(
    laser_x_px: float,
    laser_y_px: float,
    params: SharedControlParams,
) -> tuple[float, float, float]:
    radius = math.hypot(laser_x_px, laser_y_px)
    if radius <= 1e-9:
        return 0.0, 0.0, radius

    if radius <= params.free_radius_px:
        magnitude = 0.0
    elif radius <= params.safe_zone_radius_px and params.buffer_width_px > 1e-9:
        magnitude = params.max_force_n * (radius - params.free_radius_px) / params.buffer_width_px
    else:
        magnitude = params.max_force_n

    unit_x = laser_x_px / radius
    unit_y = laser_y_px / radius
    return -magnitude * unit_x, -magnitude * unit_y, radius


def compute_target_offset(
    joy_x: float,
    joy_y: float,
    laser_x_px: float,
    laser_y_px: float,
    force_x: float,
    force_y: float,
    radius_px: float,
    params: SharedControlParams,
) -> OffsetComputation:
    desired_offset_x = clip(
        joy_x * params.follow_scale_x_m,
        -params.max_offset_x_m,
        params.max_offset_x_m,
    )
    desired_offset_y = clip(
        joy_y * params.follow_scale_y_m,
        -params.max_offset_y_m,
        params.max_offset_y_m,
    )

    virtual_corr_x = clip(
        force_x * params.virtual_force_gain_x,
        -params.max_offset_x_m,
        params.max_offset_x_m,
    )
    virtual_corr_y = clip(
        force_y * params.virtual_force_gain_y,
        -params.max_offset_y_m,
        params.max_offset_y_m,
    )

    force_norm = math.hypot(force_x, force_y)
    alpha = clip(
        (force_norm - params.authority_force_start_n)
        / (params.authority_full_clamped - params.authority_force_start_n),
        0.0,
        1.0,
    )

    laser_recenter_x = 0.0
    laser_recenter_y = 0.0
    hard_recenter_active = False
    radius_norm = radius_px / max(params.safe_zone_radius_px, 1e-9)
    if radius_norm > 1.0 + 1e-6:
        hard_recenter_active = True
        outside_ratio = clip(radius_norm - 1.0, 0.0, 1.0)
        hard_push_ratio = clip(params.laser_recenter_gain + outside_ratio, 0.0, 1.0)
        unit_x = laser_x_px / max(radius_px, 1e-9)
        unit_y = laser_y_px / max(radius_px, 1e-9)
        laser_recenter_x = -unit_x * params.max_offset_x_m * hard_push_ratio
        laser_recenter_y = -unit_y * params.max_offset_y_m * hard_push_ratio
        safety_target_x = laser_recenter_x
        safety_target_y = laser_recenter_y
        target_offset_x = safety_target_x
        target_offset_y = safety_target_y
        alpha = 1.0
    else:
        safety_target_x = clip(
            virtual_corr_x,
            -params.max_offset_x_m,
            params.max_offset_x_m,
        )
        safety_target_y = clip(
            virtual_corr_y,
            -params.max_offset_y_m,
            params.max_offset_y_m,
        )

        target_offset_x = clip(
            (1.0 - alpha) * desired_offset_x + alpha * safety_target_x,
            -params.max_offset_x_m,
            params.max_offset_x_m,
        )
        target_offset_y = clip(
            (1.0 - alpha) * desired_offset_y + alpha * safety_target_y,
            -params.max_offset_y_m,
            params.max_offset_y_m,
        )

    return OffsetComputation(
        desired_offset_xy=(desired_offset_x, desired_offset_y),
        virtual_correction_xy=(virtual_corr_x, virtual_corr_y),
        laser_recenter_xy=(laser_recenter_x, laser_recenter_y),
        safety_target_xy=(safety_target_x, safety_target_y),
        target_offset_xy=(target_offset_x, target_offset_y),
        alpha=alpha,
        force_norm=force_norm,
        hard_recenter_active=hard_recenter_active,
    )
