from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Iterable

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RELEVANT_TOPICS = {
    '/joy_position',
    '/virtual_fixture_force',
    '/laser_position',
    '/shared_control_feedback',
}


@dataclass
class BagMetrics:
    trial_id: str
    bag_path: str
    condition: str
    task: str
    repeat: str
    duration_s: float
    safe_radius_px: float
    free_radius_px: float
    max_laser_px: float
    max_overshoot_px: float
    oob_duration_s: float
    first_oob_time_s: float | None
    recovery_time_s: float | None
    max_alpha: float
    max_feedback_norm: float
    max_virtual_force_n: float
    max_joy_input_norm: float
    max_joy_eff_norm: float
    hard_return_duration_s: float
    first_hard_return_time_s: float | None
    boundary_crossings: int

    def to_dict(self) -> dict[str, object]:
        return {
            'trial_id': self.trial_id,
            'bag_path': self.bag_path,
            'condition': self.condition,
            'task': self.task,
            'repeat': self.repeat,
            'duration_s': round(self.duration_s, 4),
            'completion_time_s': round(self.duration_s, 4),
            'safe_radius_px': round(self.safe_radius_px, 3),
            'free_radius_px': round(self.free_radius_px, 3),
            'max_laser_px': round(self.max_laser_px, 3),
            'max_overshoot_px': round(self.max_overshoot_px, 3),
            'oob_duration_s': round(self.oob_duration_s, 4),
            'first_oob_time_s': '' if self.first_oob_time_s is None else round(self.first_oob_time_s, 4),
            'recovery_time_s': '' if self.recovery_time_s is None else round(self.recovery_time_s, 4),
            'max_alpha': round(self.max_alpha, 4),
            'max_feedback_norm': round(self.max_feedback_norm, 4),
            'max_virtual_force_n': round(self.max_virtual_force_n, 4),
            'max_joy_input_norm': round(self.max_joy_input_norm, 4),
            'max_joy_eff_norm': round(self.max_joy_eff_norm, 4),
            'hard_return_duration_s': round(self.hard_return_duration_s, 4),
            'first_hard_return_time_s': ''
            if self.first_hard_return_time_s is None
            else round(self.first_hard_return_time_s, 4),
            'boundary_crossings': self.boundary_crossings,
        }


def clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def shaped_input(value: float, exponent: float) -> float:
    clipped = clip(value, -1.0, 1.0)
    return math.copysign(abs(clipped) ** exponent, clipped)


def compute_effective_joy_norm(
    joy_xy: tuple[float, float],
    feedback_xy: tuple[float, float],
    exponent: float,
    feedback_scale: float,
) -> float:
    effective_x = clip(
        shaped_input(joy_xy[0], exponent) + feedback_scale * feedback_xy[0],
        -1.0,
        1.0,
    )
    effective_y = clip(
        shaped_input(joy_xy[1], exponent) + feedback_scale * feedback_xy[1],
        -1.0,
        1.0,
    )
    return math.hypot(effective_x, effective_y)


def discover_bag_dirs(inputs: Iterable[str]) -> list[Path]:
    bag_dirs: list[Path] = []
    for raw_path in inputs:
        path = Path(raw_path).expanduser().resolve()
        if not path.exists():
            continue
        if path.is_dir() and (path / 'metadata.yaml').exists():
            bag_dirs.append(path)
            continue
        if path.is_dir():
            for metadata_path in sorted(path.rglob('metadata.yaml')):
                bag_dirs.append(metadata_path.parent)
    unique_dirs = sorted({bag_dir for bag_dir in bag_dirs})
    return unique_dirs


def parse_trial_fields(bag_dir: Path) -> tuple[str, str, str, str]:
    trial_id = bag_dir.name
    match = re.match(r'^C(?P<condition>[^_]+)_T(?P<task>[A-Za-z]+)_R(?P<repeat>\d+)$', trial_id)
    if not match:
        return trial_id, '', '', ''
    return (
        trial_id,
        match.group('condition'),
        match.group('task'),
        match.group('repeat'),
    )


def _load_topic_types(reader: rosbag2_py.SequentialReader) -> dict[str, object]:
    topic_types: dict[str, object] = {}
    for topic_info in reader.get_all_topics_and_types():
        if topic_info.name in RELEVANT_TOPICS:
            topic_types[topic_info.name] = get_message(topic_info.type)
    return topic_types


def analyze_bag(
    bag_dir: Path,
    storage_id: str,
    laser_input_exponent: float,
    feedback_scale: float,
    fallback_safe_radius_px: float,
) -> BagMetrics:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)
    topic_types = _load_topic_types(reader)

    trial_id, condition, task, repeat = parse_trial_fields(bag_dir)

    bag_start_ns: int | None = None
    bag_end_ns: int | None = None

    safe_radius_px = fallback_safe_radius_px
    free_radius_px = max(0.0, fallback_safe_radius_px - 60.0)
    max_laser_px = 0.0
    max_alpha = 0.0
    max_feedback_norm = 0.0
    max_virtual_force_n = 0.0
    max_joy_input_norm = 0.0
    max_joy_eff_norm = 0.0
    boundary_crossings = 0

    latest_joy_xy = (0.0, 0.0)
    latest_feedback_xy = (0.0, 0.0)

    first_oob_ns: int | None = None
    recovery_time_s: float | None = None
    oob_duration_ns = 0
    prev_laser_state_oob = False
    prev_laser_time_ns: int | None = None

    first_hard_return_ns: int | None = None
    hard_return_duration_ns = 0
    prev_hard_return = False
    prev_feedback_time_ns: int | None = None

    while reader.has_next():
        topic_name, serialized_data, timestamp_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = timestamp_ns
        bag_end_ns = timestamp_ns

        if topic_name not in topic_types:
            continue

        msg = deserialize_message(serialized_data, topic_types[topic_name])
        values = list(getattr(msg, 'data', []))

        if topic_name == '/laser_position' and len(values) >= 7:
            radius_px = float(values[4])
            free_radius_px = float(values[5])
            safe_radius_px = float(values[6])
            max_laser_px = max(max_laser_px, radius_px)
            is_oob = radius_px > safe_radius_px + 1e-6

            if prev_laser_time_ns is not None and prev_laser_state_oob:
                oob_duration_ns += max(0, timestamp_ns - prev_laser_time_ns)

            if is_oob and not prev_laser_state_oob:
                boundary_crossings += 1
                if first_oob_ns is None:
                    first_oob_ns = timestamp_ns

            if first_oob_ns is not None and recovery_time_s is None and prev_laser_state_oob and not is_oob:
                recovery_time_s = (timestamp_ns - first_oob_ns) / 1e9

            prev_laser_state_oob = is_oob
            prev_laser_time_ns = timestamp_ns

        elif topic_name == '/shared_control_feedback' and len(values) >= 4:
            feedback_x = float(values[0])
            feedback_y = float(values[1])
            alpha = float(values[2])
            hard_return = float(values[3]) > 0.5

            latest_feedback_xy = (feedback_x, feedback_y)
            max_alpha = max(max_alpha, alpha)
            max_feedback_norm = max(max_feedback_norm, math.hypot(feedback_x, feedback_y))
            max_joy_eff_norm = max(
                max_joy_eff_norm,
                compute_effective_joy_norm(
                    latest_joy_xy,
                    latest_feedback_xy,
                    laser_input_exponent,
                    feedback_scale,
                ),
            )

            if prev_feedback_time_ns is not None and prev_hard_return:
                hard_return_duration_ns += max(0, timestamp_ns - prev_feedback_time_ns)

            if hard_return and first_hard_return_ns is None:
                first_hard_return_ns = timestamp_ns

            prev_hard_return = hard_return
            prev_feedback_time_ns = timestamp_ns

        elif topic_name == '/virtual_fixture_force' and len(values) >= 2:
            max_virtual_force_n = max(
                max_virtual_force_n,
                math.hypot(float(values[0]), float(values[1])),
            )

        elif topic_name == '/joy_position' and len(values) >= 2:
            joy_x = float(values[0])
            joy_y = float(values[1])
            latest_joy_xy = (joy_x, joy_y)
            max_joy_input_norm = max(max_joy_input_norm, math.hypot(joy_x, joy_y))
            max_joy_eff_norm = max(
                max_joy_eff_norm,
                compute_effective_joy_norm(
                    latest_joy_xy,
                    latest_feedback_xy,
                    laser_input_exponent,
                    feedback_scale,
                ),
            )

    if bag_start_ns is None or bag_end_ns is None:
        raise RuntimeError(f'未在 bag 中读取到有效消息: {bag_dir}')

    if prev_laser_state_oob and prev_laser_time_ns is not None:
        oob_duration_ns += max(0, bag_end_ns - prev_laser_time_ns)
    if prev_hard_return and prev_feedback_time_ns is not None:
        hard_return_duration_ns += max(0, bag_end_ns - prev_feedback_time_ns)

    duration_s = max(0.0, (bag_end_ns - bag_start_ns) / 1e9)
    max_overshoot_px = max(0.0, max_laser_px - safe_radius_px)
    first_oob_time_s = None if first_oob_ns is None else (first_oob_ns - bag_start_ns) / 1e9
    first_hard_return_time_s = (
        None if first_hard_return_ns is None else (first_hard_return_ns - bag_start_ns) / 1e9
    )

    return BagMetrics(
        trial_id=trial_id,
        bag_path=str(bag_dir),
        condition=condition,
        task=task,
        repeat=repeat,
        duration_s=duration_s,
        safe_radius_px=safe_radius_px,
        free_radius_px=free_radius_px,
        max_laser_px=max_laser_px,
        max_overshoot_px=max_overshoot_px,
        oob_duration_s=oob_duration_ns / 1e9,
        first_oob_time_s=first_oob_time_s,
        recovery_time_s=recovery_time_s,
        max_alpha=max_alpha,
        max_feedback_norm=max_feedback_norm,
        max_virtual_force_n=max_virtual_force_n,
        max_joy_input_norm=max_joy_input_norm,
        max_joy_eff_norm=max_joy_eff_norm,
        hard_return_duration_s=hard_return_duration_ns / 1e9,
        first_hard_return_time_s=first_hard_return_time_s,
        boundary_crossings=boundary_crossings,
    )


def write_csv(rows: list[dict[str, object]], output_path: Path) -> None:
    if not rows:
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with output_path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def merge_into_template(
    template_csv: Path,
    metrics_rows: list[dict[str, object]],
    output_csv: Path,
) -> None:
    metrics_by_trial = {str(row['trial_id']): row for row in metrics_rows}
    with template_csv.open(newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        template_rows = list(reader)
        fieldnames = list(reader.fieldnames or [])

    for metric_field in metrics_rows[0].keys():
        if metric_field not in fieldnames:
            fieldnames.append(metric_field)

    merged_rows: list[dict[str, object]] = []
    seen_trials: set[str] = set()
    for row in template_rows:
        trial_id = row.get('trial_id', '')
        metric_row = metrics_by_trial.get(trial_id)
        if metric_row is not None:
            for key, value in metric_row.items():
                row[key] = value
            if not row.get('bag_path'):
                row['bag_path'] = metric_row['bag_path']
            if not row.get('condition'):
                row['condition'] = metric_row['condition']
            if not row.get('task'):
                row['task'] = metric_row['task']
            if not row.get('repeat'):
                row['repeat'] = metric_row['repeat']
            seen_trials.add(trial_id)
        merged_rows.append(row)

    for trial_id, metric_row in metrics_by_trial.items():
        if trial_id in seen_trials:
            continue
        merged_rows.append({key: metric_row.get(key, '') for key in fieldnames})

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(merged_rows)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='分析共享控制实验 rosbag，并导出核心对比指标 CSV。',
    )
    parser.add_argument(
        'inputs',
        nargs='+',
        help='一个或多个 bag 目录，或包含多个 bag 目录的父目录。',
    )
    parser.add_argument(
        '--output-csv',
        default='shared_control_experiment_analysis.csv',
        help='分析结果 CSV 输出路径。',
    )
    parser.add_argument(
        '--template-csv',
        default='',
        help='可选：把结果回填进实验记录模板 CSV。',
    )
    parser.add_argument(
        '--merged-output-csv',
        default='',
        help='可选：模板回填后的输出 CSV 路径。',
    )
    parser.add_argument(
        '--storage-id',
        default='sqlite3',
        help='rosbag storage id，默认 sqlite3。',
    )
    parser.add_argument(
        '--laser-input-exponent',
        type=float,
        default=1.6,
        help='用于估算 Joy Eff 的人手输入非线性压缩指数。',
    )
    parser.add_argument(
        '--shared-feedback-scale',
        type=float,
        default=1.0,
        help='用于估算 Joy Eff 的反馈缩放系数。',
    )
    parser.add_argument(
        '--fallback-safe-radius-px',
        type=float,
        default=220.0,
        help='当 bag 中缺少安全半径字段时使用的默认值。',
    )
    return parser


def main(args: list[str] | None = None) -> None:
    parser = build_arg_parser()
    parsed = parser.parse_args(args=args)

    bag_dirs = discover_bag_dirs(parsed.inputs)
    if not bag_dirs:
        raise SystemExit('未发现可分析的 rosbag 目录（需要包含 metadata.yaml）。')

    metrics_rows: list[dict[str, object]] = []
    for bag_dir in bag_dirs:
        metrics = analyze_bag(
            bag_dir=bag_dir,
            storage_id=parsed.storage_id,
            laser_input_exponent=parsed.laser_input_exponent,
            feedback_scale=parsed.shared_feedback_scale,
            fallback_safe_radius_px=parsed.fallback_safe_radius_px,
        )
        metrics_rows.append(metrics.to_dict())

    metrics_rows.sort(key=lambda row: str(row['trial_id']))
    output_csv = Path(parsed.output_csv).expanduser()
    write_csv(metrics_rows, output_csv)
    print(f'已输出分析结果: {output_csv}')

    if parsed.template_csv:
        template_csv = Path(parsed.template_csv).expanduser()
        merged_output_csv = (
            Path(parsed.merged_output_csv).expanduser()
            if parsed.merged_output_csv
            else output_csv.with_name(output_csv.stem + '_merged.csv')
        )
        merge_into_template(template_csv, metrics_rows, merged_output_csv)
        print(f'已输出模板回填结果: {merged_output_csv}')


if __name__ == '__main__':
    main()
