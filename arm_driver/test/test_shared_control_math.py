"""Unit tests for the shared-control law; no ROS or hardware required."""

import math

from arm_driver.shared_control_math import (
    compute_target_offset,
    compute_virtual_force,
    integrate_laser,
    SharedControlParams,
)

PARAMS = SharedControlParams()


def offset_at(laser_x_px, laser_y_px, joy_x=0.0, joy_y=0.0, params=PARAMS):
    force_x, force_y, radius = compute_virtual_force(laser_x_px, laser_y_px, params)
    return compute_target_offset(
        joy_x, joy_y, laser_x_px, laser_y_px, force_x, force_y, radius, params)


def test_no_force_inside_free_zone():
    force_x, force_y, _ = compute_virtual_force(PARAMS.free_radius_px - 1.0, 0.0, PARAMS)
    assert force_x == 0.0 and force_y == 0.0


def test_force_ramps_across_buffer_and_points_inward():
    mid = PARAMS.free_radius_px + PARAMS.buffer_width_px / 2.0
    force_x, force_y, _ = compute_virtual_force(mid, 0.0, PARAMS)
    assert math.isclose(force_x, -PARAMS.max_force_n / 2.0)
    assert force_y == 0.0
    edge_x, _, _ = compute_virtual_force(PARAMS.safe_zone_radius_px, 0.0, PARAMS)
    assert math.isclose(edge_x, -PARAMS.max_force_n)


def test_operator_has_full_authority_without_fixture_force():
    result = offset_at(0.0, 0.0, joy_x=0.5, joy_y=-0.5)
    assert result.alpha == 0.0
    assert not result.hard_recenter_active
    assert result.target_offset_xy == result.desired_offset_xy


def test_alpha_blends_operator_input_and_correction():
    # A quarter of the way into the buffer the force is 2 N, between the authority thresholds.
    result = offset_at(PARAMS.free_radius_px + PARAMS.buffer_width_px / 4.0, 0.0, joy_x=1.0)
    assert 0.0 < result.alpha < 1.0
    (desired_x, _), (safety_x, _) = result.desired_offset_xy, result.safety_target_xy
    expected_x = (1.0 - result.alpha) * desired_x + result.alpha * safety_x
    assert math.isclose(result.target_offset_xy[0], expected_x)


def test_machine_takes_full_authority_outside_boundary():
    result = offset_at(PARAMS.safe_zone_radius_px + 30.0, 0.0, joy_x=1.0)
    assert result.alpha == 1.0
    assert result.hard_recenter_active
    # The correction ignores the outward joystick input and pushes back toward the centre.
    assert result.target_offset_xy[0] < 0.0


def test_target_offset_stays_within_workspace_limits():
    for laser_x, joy_x in ((0.0, 10.0), (PARAMS.safe_zone_radius_px * 2.0, -10.0)):
        result = offset_at(laser_x, 0.0, joy_x=joy_x, joy_y=joy_x)
        assert abs(result.target_offset_xy[0]) <= PARAMS.max_offset_x_m + 1e-12
        assert abs(result.target_offset_xy[1]) <= PARAMS.max_offset_y_m + 1e-12


def test_laser_is_limited_to_display_radius():
    laser_x, laser_y = integrate_laser((PARAMS.laser_limit_radius_px, 0.0), 1.0, 1.0, 1.0, PARAMS)
    assert math.hypot(laser_x, laser_y) <= PARAMS.laser_limit_radius_px + 1e-9
