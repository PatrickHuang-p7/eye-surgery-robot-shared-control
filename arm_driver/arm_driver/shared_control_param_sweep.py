from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from dataclasses import replace
import itertools
import math
from pathlib import Path
from typing import Callable

try:
    from .shared_control_math import SharedControlParams
    from .shared_control_math import clip
    from .shared_control_math import compute_target_offset
    from .shared_control_math import compute_virtual_force
    from .shared_control_math import integrate_laser
except ImportError:  # pragma: no cover - 兼容直接 python 执行文件
    from shared_control_math import SharedControlParams
    from shared_control_math import clip
    from shared_control_math import compute_target_offset
    from shared_control_math import compute_virtual_force
    from shared_control_math import integrate_laser


def _unit_xy(x: float, y: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= 1e-9:
        return 0.0, 0.0
    return x / norm, y / norm


def _parse_float_list(raw: str) -> list[float]:
    return [float(item.strip()) for item in raw.split(',') if item.strip()]


@dataclass(frozen=True)
class SimulatorConfig:
    dt_s: float = 1.0 / 60.0
    arm_response_tau_s: float = 0.18
    joystick_coupling_gain: float = 1.10
    saturation_threshold_ratio: float = 0.95
    preemptive_alpha_threshold: float = 0.25


@dataclass(frozen=True)
class Scenario:
    name: str
    duration_s: float
    weight: float
    user_input_fn: Callable[[float], tuple[float, float]]


@dataclass(frozen=True)
class ScenarioMetrics:
    outside_time_s: float
    peak_overshoot_px: float
    return_time_s: float
    boundary_crossings: int
    saturation_ratio: float
    preemptive_alpha_ratio: float
    mean_alpha_inside: float
    mean_alpha_outside: float
    mean_machine_opposition: float
    mean_user_effective_gap: float
    score: float


def build_scenarios() -> list[Scenario]:
    diag_xy = _unit_xy(1.0, 1.0)
    slide_xy = _unit_xy(0.55, 0.90)

    return [
        Scenario(
            name='push_hold_x',
            duration_s=3.0,
            weight=1.20,
            user_input_fn=lambda t: (1.0, 0.0),
        ),
        Scenario(
            name='push_release_x',
            duration_s=3.8,
            weight=1.00,
            user_input_fn=lambda t: (1.0, 0.0) if t < 1.6 else (0.0, 0.0),
        ),
        Scenario(
            name='push_release_diag',
            duration_s=4.0,
            weight=1.00,
            user_input_fn=lambda t: diag_xy if t < 1.8 else (0.0, 0.0),
        ),
        Scenario(
            name='edge_slide',
            duration_s=4.2,
            weight=1.10,
            user_input_fn=lambda t: (
                (1.0, 0.0)
                if t < 1.2
                else slide_xy
                if t < 3.2
                else (0.0, 0.0)
            ),
        ),
        Scenario(
            name='pulsed_nudges',
            duration_s=4.0,
            weight=0.90,
            user_input_fn=lambda t: (
                (0.85, 0.0)
                if (t % 1.2) < 0.55
                else diag_xy
                if (t % 1.2) < 0.85
                else (0.0, 0.0)
            ),
        ),
    ]


def _clip_joy_vector(x: float, y: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= 1.0:
        return x, y
    return x / norm, y / norm


def _compute_metrics_score(metrics: ScenarioMetrics) -> float:
    return (
        1.00 * metrics.outside_time_s
        + 0.06 * metrics.peak_overshoot_px
        + 0.80 * metrics.return_time_s
        + 0.45 * metrics.boundary_crossings
        + 2.40 * metrics.saturation_ratio
        + 2.80 * metrics.preemptive_alpha_ratio
        + 1.20 * metrics.mean_alpha_inside
        + 0.80 * metrics.mean_machine_opposition
        + 0.60 * metrics.mean_user_effective_gap
    )


def simulate_scenario(
    params: SharedControlParams,
    scenario: Scenario,
    sim: SimulatorConfig,
) -> ScenarioMetrics:
    steps = max(1, int(math.ceil(scenario.duration_s / sim.dt_s)))
    laser_xy_px = (0.0, 0.0)
    machine_offset_xy = (0.0, 0.0)

    outside_time_s = 0.0
    peak_overshoot_px = 0.0
    first_outside_t = None
    return_time_s = None
    boundary_crossings = 0
    prev_outside = False
    saturation_count = 0
    inside_alpha_sum = 0.0
    inside_count = 0
    outside_alpha_sum = 0.0
    outside_count = 0
    preemptive_alpha_count = 0
    opposition_sum = 0.0
    user_gap_sum = 0.0
    active_user_count = 0

    response_blend = 1.0 - math.exp(-sim.dt_s / max(sim.arm_response_tau_s, 1e-6))

    for step_idx in range(steps):
        t = step_idx * sim.dt_s
        user_joy_x, user_joy_y = scenario.user_input_fn(t)

        machine_feedback_x = sim.joystick_coupling_gain * (
            machine_offset_xy[0] / max(params.max_offset_x_m, 1e-9)
        )
        machine_feedback_y = sim.joystick_coupling_gain * (
            machine_offset_xy[1] / max(params.max_offset_y_m, 1e-9)
        )
        effective_joy_x, effective_joy_y = _clip_joy_vector(
            user_joy_x + machine_feedback_x,
            user_joy_y + machine_feedback_y,
        )

        laser_xy_px = integrate_laser(
            laser_xy_px,
            effective_joy_x,
            effective_joy_y,
            sim.dt_s,
            params,
        )
        laser_x_px, laser_y_px = laser_xy_px
        force_x, force_y, radius_px = compute_virtual_force(laser_x_px, laser_y_px, params)
        offset_result = compute_target_offset(
            user_joy_x,
            user_joy_y,
            laser_x_px,
            laser_y_px,
            force_x,
            force_y,
            radius_px,
            params,
        )

        machine_offset_xy = (
            machine_offset_xy[0]
            + response_blend * (offset_result.target_offset_xy[0] - machine_offset_xy[0]),
            machine_offset_xy[1]
            + response_blend * (offset_result.target_offset_xy[1] - machine_offset_xy[1]),
        )

        free_zone = radius_px <= params.free_radius_px
        outside_boundary = radius_px > params.safe_zone_radius_px
        if outside_boundary:
            outside_time_s += sim.dt_s
            outside_count += 1
            outside_alpha_sum += offset_result.alpha
            peak_overshoot_px = max(
                peak_overshoot_px,
                radius_px - params.safe_zone_radius_px,
            )
            if first_outside_t is None:
                first_outside_t = t
        elif free_zone:
            inside_count += 1
            inside_alpha_sum += offset_result.alpha
            if offset_result.alpha > sim.preemptive_alpha_threshold:
                preemptive_alpha_count += 1

        if step_idx > 0 and outside_boundary != prev_outside:
            boundary_crossings += 1
        prev_outside = outside_boundary

        if (
            first_outside_t is not None
            and return_time_s is None
            and (not outside_boundary)
            and t > first_outside_t
        ):
            return_time_s = t - first_outside_t

        if (
            abs(offset_result.target_offset_xy[0]) >= sim.saturation_threshold_ratio * params.max_offset_x_m
            or abs(offset_result.target_offset_xy[1]) >= sim.saturation_threshold_ratio * params.max_offset_y_m
        ):
            saturation_count += 1

        user_mag = math.hypot(user_joy_x, user_joy_y)
        if user_mag > 1e-6:
            active_user_count += 1
            opposition = max(
                0.0,
                -(user_joy_x * machine_feedback_x + user_joy_y * machine_feedback_y) / user_mag,
            )
            gap = math.hypot(user_joy_x - effective_joy_x, user_joy_y - effective_joy_y)
            opposition_sum += opposition
            user_gap_sum += gap

    if first_outside_t is None:
        return_time_value = 0.0
    elif return_time_s is None:
        return_time_value = scenario.duration_s - first_outside_t
    else:
        return_time_value = return_time_s

    metrics = ScenarioMetrics(
        outside_time_s=outside_time_s,
        peak_overshoot_px=peak_overshoot_px,
        return_time_s=return_time_value,
        boundary_crossings=boundary_crossings,
        saturation_ratio=saturation_count / steps,
        preemptive_alpha_ratio=preemptive_alpha_count / max(inside_count, 1),
        mean_alpha_inside=inside_alpha_sum / max(inside_count, 1),
        mean_alpha_outside=outside_alpha_sum / max(outside_count, 1),
        mean_machine_opposition=opposition_sum / max(active_user_count, 1),
        mean_user_effective_gap=user_gap_sum / max(active_user_count, 1),
        score=0.0,
    )
    return ScenarioMetrics(**{**metrics.__dict__, 'score': _compute_metrics_score(metrics)})


def aggregate_results(
    params: SharedControlParams,
    scenarios: list[Scenario],
    sim: SimulatorConfig,
) -> dict:
    scenario_metrics = {
        scenario.name: simulate_scenario(params, scenario, sim) for scenario in scenarios
    }
    total_weight = sum(scenario.weight for scenario in scenarios)
    weighted_score = sum(
        scenario.weight * scenario_metrics[scenario.name].score for scenario in scenarios
    ) / total_weight

    row = {
        'score': weighted_score,
        'laser_recenter_gain': params.laser_recenter_gain,
        'authority_force_start_n': params.authority_force_start_n,
        'authority_force_full_n': params.authority_force_full_n,
        'virtual_force_gain': params.virtual_force_gain_x,
        'follow_scale': params.follow_scale_x_m,
        'free_radius_px': params.free_radius_px,
    }

    worst_peak_overshoot_px = 0.0
    worst_return_time_s = 0.0
    worst_outside_time_s = 0.0
    worst_preemptive_alpha_ratio = 0.0
    for name, metrics in scenario_metrics.items():
        row[f'{name}_outside_time_s'] = metrics.outside_time_s
        row[f'{name}_peak_overshoot_px'] = metrics.peak_overshoot_px
        row[f'{name}_return_time_s'] = metrics.return_time_s
        row[f'{name}_boundary_crossings'] = metrics.boundary_crossings
        row[f'{name}_saturation_ratio'] = metrics.saturation_ratio
        row[f'{name}_preemptive_alpha_ratio'] = metrics.preemptive_alpha_ratio
        row[f'{name}_mean_alpha_inside'] = metrics.mean_alpha_inside
        row[f'{name}_mean_alpha_outside'] = metrics.mean_alpha_outside
        row[f'{name}_mean_machine_opposition'] = metrics.mean_machine_opposition
        row[f'{name}_mean_user_effective_gap'] = metrics.mean_user_effective_gap
        row[f'{name}_score'] = metrics.score

        worst_peak_overshoot_px = max(worst_peak_overshoot_px, metrics.peak_overshoot_px)
        worst_return_time_s = max(worst_return_time_s, metrics.return_time_s)
        worst_outside_time_s = max(worst_outside_time_s, metrics.outside_time_s)
        worst_preemptive_alpha_ratio = max(
            worst_preemptive_alpha_ratio,
            metrics.preemptive_alpha_ratio,
        )

    row['worst_peak_overshoot_px'] = worst_peak_overshoot_px
    row['worst_return_time_s'] = worst_return_time_s
    row['worst_outside_time_s'] = worst_outside_time_s
    row['worst_preemptive_alpha_ratio'] = worst_preemptive_alpha_ratio
    return row


def build_param_grid(args: argparse.Namespace) -> list[SharedControlParams]:
    base = SharedControlParams()
    laser_recenter_gains = _parse_float_list(args.laser_recenter_gains)
    authority_force_starts = _parse_float_list(args.authority_force_starts)
    authority_force_fulls = _parse_float_list(args.authority_force_fulls)
    virtual_force_gains = _parse_float_list(args.virtual_force_gains)
    follow_scales = _parse_float_list(args.follow_scales)

    candidates = []
    for (
        laser_recenter_gain,
        authority_force_start_n,
        authority_force_full_n,
        virtual_force_gain,
        follow_scale,
    ) in itertools.product(
        laser_recenter_gains,
        authority_force_starts,
        authority_force_fulls,
        virtual_force_gains,
        follow_scales,
    ):
        if authority_force_full_n <= authority_force_start_n:
            continue
        candidates.append(
            replace(
                base,
                laser_recenter_gain=laser_recenter_gain,
                authority_force_start_n=authority_force_start_n,
                authority_force_full_n=authority_force_full_n,
                virtual_force_gain_x=virtual_force_gain,
                virtual_force_gain_y=virtual_force_gain,
                follow_scale_x_m=follow_scale,
                follow_scale_y_m=follow_scale,
            )
        )
    return candidates


def write_csv(rows: list[dict], output_path: Path) -> None:
    if not rows:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with output_path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_top_results(rows: list[dict], top_k: int) -> None:
    print('')
    print('Top Parameter Sets')
    print('rank  score   recenter  alpha_start/full  vf_gain   follow  worst_over  worst_return  pre_alpha')
    for idx, row in enumerate(rows[:top_k], start=1):
        print(
            f'{idx:>4}  '
            f'{row["score"]:>6.3f}  '
            f'{row["laser_recenter_gain"]:>8.3f}  '
            f'{row["authority_force_start_n"]:>4.1f}/{row["authority_force_full_n"]:<4.1f}      '
            f'{row["virtual_force_gain"]:>7.4f}  '
            f'{row["follow_scale"]:>6.3f}  '
            f'{row["worst_peak_overshoot_px"]:>10.1f}  '
            f'{row["worst_return_time_s"]:>12.2f}  '
            f'{row["worst_preemptive_alpha_ratio"]:>8.2f}'
        )


def print_top_range(rows: list[dict], top_k: int) -> None:
    top_rows = rows[:top_k]
    if not top_rows:
        return
    keys = [
        'laser_recenter_gain',
        'authority_force_start_n',
        'authority_force_full_n',
        'virtual_force_gain',
        'follow_scale',
    ]
    print('')
    print(f'Top-{top_k} Envelope')
    for key in keys:
        values = [float(row[key]) for row in top_rows]
        print(f'- {key}: {min(values):.4f} ~ {max(values):.4f}')


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Offline parameter sweep for shared-control tuning.',
    )
    parser.add_argument(
        '--output-csv',
        default='shared_control_param_sweep.csv',
        help='Path to write CSV results.',
    )
    parser.add_argument('--top-k', type=int, default=10)
    parser.add_argument(
        '--laser-recenter-gains',
        default='0.85,1.00',
        help='Comma-separated sweep values.',
    )
    parser.add_argument(
        '--authority-force-starts',
        default='1.0,1.5',
        help='Comma-separated sweep values.',
    )
    parser.add_argument(
        '--authority-force-fulls',
        default='4.0',
        help='Comma-separated sweep values.',
    )
    parser.add_argument(
        '--virtual-force-gains',
        default='0.0009',
        help='Comma-separated sweep values.',
    )
    parser.add_argument(
        '--follow-scales',
        default='0.008,0.009,0.010',
        help='Comma-separated sweep values.',
    )
    parser.add_argument('--dt-s', type=float, default=1.0 / 60.0)
    parser.add_argument('--arm-response-tau-s', type=float, default=0.18)
    parser.add_argument('--joystick-coupling-gain', type=float, default=1.10)
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    sim = SimulatorConfig(
        dt_s=args.dt_s,
        arm_response_tau_s=args.arm_response_tau_s,
        joystick_coupling_gain=args.joystick_coupling_gain,
    )
    scenarios = build_scenarios()
    param_grid = build_param_grid(args)

    print(
        f'Running sweep: {len(param_grid)} parameter sets | '
        f'{len(scenarios)} scenarios | '
        f'dt={sim.dt_s:.4f}s | tau={sim.arm_response_tau_s:.3f}s | '
        f'coupling={sim.joystick_coupling_gain:.2f}'
    )

    rows = [aggregate_results(params, scenarios, sim) for params in param_grid]
    rows.sort(key=lambda item: item['score'])

    output_path = Path(args.output_csv).expanduser()
    write_csv(rows, output_path)
    print_top_results(rows, args.top_k)
    print_top_range(rows, args.top_k)
    print('')
    print(f'CSV written to: {output_path}')


if __name__ == '__main__':
    main()
