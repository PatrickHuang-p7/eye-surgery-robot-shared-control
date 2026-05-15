from __future__ import annotations

import argparse
import csv
import shutil
from datetime import date
from pathlib import Path
import re
import time


DEFAULT_CSV_PATH = Path('/home/pq/arm_project_ws/experiment_outputs/notes/experiment_record_filled.csv')
DEFAULT_BAG_ROOT = Path('/home/pq/arm_project_ws/experiment_outputs/bag')
DEFAULT_SCREENSHOT_ROOT = Path('/home/pq/arm_project_ws/experiment_outputs/screenshot')
DEFAULT_CAPTURE_ROOT = Path('/tmp/pygame_haptic_sim_captures')
DEFAULT_PYGAME_YAML = '/home/pq/arm_project_ws/src/arm_driver/config/pygame_haptic_sim_shared_debug.yaml'
DEFAULT_BASE_YAML = '/home/pq/arm_project_ws/src/arm_driver/config/arm_control_shared_baseline.yaml'
DEFAULT_SHARED_YAML = '/home/pq/arm_project_ws/src/arm_driver/config/arm_control_shared_balanced.yaml'

FIELDNAMES = [
    'trial_id',
    'date',
    'operator_id',
    'condition',
    'task',
    'repeat',
    'arm_control_yaml',
    'pygame_yaml',
    'bag_path',
    'screenshot_path',
    'success',
    'completed_as_instructed',
    'max_laser_px',
    'max_overshoot_px',
    'oob_duration_s',
    'first_oob_time_s',
    'recovery_time_s',
    'completion_time_s',
    'max_alpha',
    'max_feedback_norm',
    'max_virtual_force_n',
    'max_joy_input_norm',
    'max_joy_eff_norm',
    'oscillation_observed',
    'slip_observed',
    'hardware_fault',
    'notes',
]


def parse_trial_id(trial_id: str) -> tuple[str, str, str]:
    match = re.match(r'^C(?P<condition>[^_]+)_T(?P<task>[A-Za-z]+)_R(?P<repeat>\d+)$', trial_id)
    if not match:
        raise ValueError(f'试验编号格式不正确: {trial_id}')
    return match.group('condition'), match.group('task'), match.group('repeat')


def default_arm_yaml(condition: str) -> str:
    if condition == 'base':
        return DEFAULT_BASE_YAML
    if condition == 'shared':
        return DEFAULT_SHARED_YAML
    return ''


def normalize_flag(value: int | None) -> str:
    if value is None:
        return ''
    return '1' if int(value) else '0'


def read_rows(csv_path: Path) -> list[dict[str, str]]:
    if not csv_path.exists():
        return []
    with csv_path.open(newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        sanitized_rows: list[dict[str, str]] = []
        for raw_row in reader:
            row = {field: '' for field in FIELDNAMES}
            for key, value in raw_row.items():
                if key in FIELDNAMES:
                    row[key] = value
            sanitized_rows.append(row)
        return sanitized_rows


def write_rows(csv_path: Path, rows: list[dict[str, str]]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
        writer.writeheader()
        sanitized_rows: list[dict[str, str]] = []
        for raw_row in rows:
            row = {field: raw_row.get(field, '') for field in FIELDNAMES}
            sanitized_rows.append(row)
        writer.writerows(sanitized_rows)


def move_latest_screenshot(target_path: Path, max_age_sec: float | None) -> str:
    captures = sorted(DEFAULT_CAPTURE_ROOT.glob('*.png'), key=lambda item: item.stat().st_mtime, reverse=True)
    if not captures:
        raise FileNotFoundError(f'未找到可移动的截图: {DEFAULT_CAPTURE_ROOT}')
    latest = captures[0]
    if max_age_sec is not None:
        age_sec = time.time() - latest.stat().st_mtime
        if age_sec > max_age_sec:
            raise FileNotFoundError(
                f'最新截图距离现在已超过 {max_age_sec:.0f} 秒，疑似不是本条 trial 的截图: {latest}'
            )
    target_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(latest), str(target_path))
    return str(target_path)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='把单条实验记录写入 experiment_record_filled.csv。')
    parser.add_argument('trial_id', help='例如 Cbase_TU_R01 或 Cshared_TCW_R03')
    parser.add_argument('--csv-path', default=str(DEFAULT_CSV_PATH), help='记录表 CSV 路径')
    parser.add_argument('--date', default=date.today().isoformat(), help='日期，默认今天')
    parser.add_argument('--operator-id', default='P1', help='操作者编号，默认 P1')
    parser.add_argument('--arm-control-yaml', default='', help='可选：覆盖自动推断的 arm_control yaml 路径')
    parser.add_argument('--pygame-yaml', default=DEFAULT_PYGAME_YAML, help='pygame yaml 路径')
    parser.add_argument('--bag-path', default='', help='可选：覆盖自动推断的 bag 路径')
    parser.add_argument('--screenshot-path', default='', help='可选：截图路径')
    parser.add_argument('--capture-latest-screenshot', action='store_true', help='把 /tmp/pygame_haptic_sim_captures 下最新截图移动到标准位置')
    parser.add_argument(
        '--latest-screenshot-max-age-sec',
        type=float,
        default=300.0,
        help='配合 --capture-latest-screenshot 使用；若最新截图超过该秒数则拒绝绑定，默认 300 秒',
    )
    parser.add_argument('--success', type=int, choices=[0, 1], default=None, help='1=成功，0=失败')
    parser.add_argument('--completed-as-instructed', type=int, choices=[0, 1], default=None, help='1=按要求完成，0=未按要求完成')
    parser.add_argument('--oscillation-observed', type=int, choices=[0, 1], default=None, help='1=有明显振荡')
    parser.add_argument('--slip-observed', type=int, choices=[0, 1], default=None, help='1=夹持打滑')
    parser.add_argument('--hardware-fault', type=int, choices=[0, 1], default=None, help='1=有硬件异常')
    parser.add_argument('--notes', default='', help='简短备注，例如 normal / minor slip')
    return parser


def main(args: list[str] | None = None) -> None:
    parser = build_arg_parser()
    parsed = parser.parse_args(args=args)

    trial_id = parsed.trial_id.strip()
    condition, task, repeat = parse_trial_id(trial_id)

    csv_path = Path(parsed.csv_path).expanduser()
    bag_path = parsed.bag_path.strip() or str((DEFAULT_BAG_ROOT / trial_id).resolve())

    if parsed.capture_latest_screenshot:
        screenshot_path = move_latest_screenshot(
            (DEFAULT_SCREENSHOT_ROOT / f'{trial_id}.png').resolve(),
            parsed.latest_screenshot_max_age_sec,
        )
    else:
        screenshot_path = parsed.screenshot_path.strip()

    row_data = {
        'trial_id': trial_id,
        'date': parsed.date,
        'operator_id': parsed.operator_id,
        'condition': condition,
        'task': task,
        'repeat': repeat,
        'arm_control_yaml': parsed.arm_control_yaml.strip() or default_arm_yaml(condition),
        'pygame_yaml': parsed.pygame_yaml,
        'bag_path': bag_path,
        'screenshot_path': screenshot_path,
        'success': normalize_flag(parsed.success),
        'completed_as_instructed': normalize_flag(parsed.completed_as_instructed),
        'oscillation_observed': normalize_flag(parsed.oscillation_observed),
        'slip_observed': normalize_flag(parsed.slip_observed),
        'hardware_fault': normalize_flag(parsed.hardware_fault),
        'notes': parsed.notes,
    }

    rows = read_rows(csv_path)
    updated = False
    for row in rows:
        if row.get('trial_id') == trial_id:
            for key, value in row_data.items():
                if value != '':
                    row[key] = value
            updated = True
            break

    if not updated:
        new_row = {field: '' for field in FIELDNAMES}
        new_row.update(row_data)
        rows.append(new_row)

    rows.sort(key=lambda item: item.get('trial_id', ''))
    write_rows(csv_path, rows)
    print(f'已写入记录表: {csv_path}')
    print(f'TrialID: {trial_id}')
    print(f'Bag: {bag_path}')
    if screenshot_path:
        print(f'Screenshot: {screenshot_path}')


if __name__ == '__main__':
    main()
