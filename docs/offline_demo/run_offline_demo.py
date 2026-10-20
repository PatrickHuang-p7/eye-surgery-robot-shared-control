"""Offline demonstration of the shared-control law, without ROS or hardware.

Runs the closed-loop joystick/arm model from shared_control_param_sweep.py with the
baseline and balanced parameter sets on a scripted push-and-release input, then writes
a trajectory plot and a metrics CSV. This is a simulation of the control law, not a
record of bench trials.

Usage (from the repository root):
    python3 docs/offline_demo/run_offline_demo.py
"""

from dataclasses import replace
import csv
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'arm_driver'))

from arm_driver.shared_control_math import (  # noqa: E402
    compute_target_offset,
    compute_virtual_force,
    integrate_laser,
    SharedControlParams,
)
from arm_driver.shared_control_param_sweep import (  # noqa: E402
    _clip_joy_vector,
    build_scenarios,
    simulate_scenario,
    SimulatorConfig,
)

OUT_DIR = Path(__file__).resolve().parent
CONFIGS = ('baseline', 'balanced')
SCENARIO = 'push_release_x'


def load_params(name: str) -> SharedControlParams:
    """Map an arm_control_shared_<name>.yaml file onto SharedControlParams."""
    path = ROOT / 'arm_driver' / 'config' / f'arm_control_shared_{name}.yaml'
    p = yaml.safe_load(path.read_text())['arm_control_node']['ros__parameters']
    return replace(
        SharedControlParams(),
        max_offset_x_m=p['max_joystick_offset_x_m'],
        max_offset_y_m=p['max_joystick_offset_y_m'],
        follow_scale_x_m=p['joystick_follow_scale_x_m'],
        follow_scale_y_m=p['joystick_follow_scale_y_m'],
        virtual_force_gain_x=p['virtual_force_gain_x_m_per_n'],
        virtual_force_gain_y=p['virtual_force_gain_y_m_per_n'],
        authority_force_start_n=p['authority_force_start_n'],
        authority_force_full_n=p['authority_force_full_n'],
        laser_recenter_gain=p['laser_recenter_gain'],
    )


def trace(params: SharedControlParams, scenario, sim: SimulatorConfig) -> dict:
    """Same update as simulate_scenario(), keeping the time series for plotting."""
    steps = max(1, int(math.ceil(scenario.duration_s / sim.dt_s)))
    blend = 1.0 - math.exp(-sim.dt_s / max(sim.arm_response_tau_s, 1e-6))
    laser = (0.0, 0.0)
    machine = (0.0, 0.0)
    series = {'t': [], 'radius_px': [], 'alpha': [], 'user_x': []}
    for i in range(steps):
        t = i * sim.dt_s
        user_x, user_y = scenario.user_input_fn(t)
        feedback_x = sim.joystick_coupling_gain * machine[0] / max(params.max_offset_x_m, 1e-9)
        feedback_y = sim.joystick_coupling_gain * machine[1] / max(params.max_offset_y_m, 1e-9)
        joy_x, joy_y = _clip_joy_vector(user_x + feedback_x, user_y + feedback_y)
        laser = integrate_laser(laser, joy_x, joy_y, sim.dt_s, params)
        force_x, force_y, radius = compute_virtual_force(laser[0], laser[1], params)
        result = compute_target_offset(
            user_x, user_y, laser[0], laser[1], force_x, force_y, radius, params)
        machine = (
            machine[0] + blend * (result.target_offset_xy[0] - machine[0]),
            machine[1] + blend * (result.target_offset_xy[1] - machine[1]),
        )
        series['t'].append(t)
        series['radius_px'].append(radius)
        series['alpha'].append(result.alpha)
        series['user_x'].append(user_x)
    return series


def main() -> None:
    sim = SimulatorConfig()
    scenarios = {s.name: s for s in build_scenarios()}
    params = {name: load_params(name) for name in CONFIGS}

    rows = []
    for name in CONFIGS:
        for scenario in scenarios.values():
            m = simulate_scenario(params[name], scenario, sim)
            rows.append({
                'config': name,
                'scenario': scenario.name,
                'peak_overshoot_px': round(m.peak_overshoot_px, 2),
                'outside_time_s': round(m.outside_time_s, 3),
                'return_time_s': round(m.return_time_s, 3),
                'boundary_crossings': m.boundary_crossings,
            })
    with open(OUT_DIR / 'offline_metrics.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    fig, (ax_r, ax_a) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    safe = params['balanced'].safe_zone_radius_px
    for name, style in zip(CONFIGS, ('--', '-')):
        s = trace(params[name], scenarios[SCENARIO], sim)
        ax_r.plot(s['t'], s['radius_px'], style, label=name)
        ax_a.plot(s['t'], s['alpha'], style, label=name)
    ax_r.axhline(safe, color='grey', linewidth=0.8)
    ax_r.text(0.02, safe + 4, 'safe-zone boundary', color='grey', fontsize=8)
    ax_r.set_ylabel('laser radius (px)')
    ax_r.legend(loc='lower right')
    ax_a.set_ylabel('authority alpha')
    ax_a.set_xlabel('time (s): operator pushes +x for 1.6 s, then releases')
    fig.suptitle('Offline simulation of the control law (not bench data)', fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT_DIR / 'offline_push_release.png', dpi=150)
    for row in rows:
        print(row)


if __name__ == '__main__':
    main()
