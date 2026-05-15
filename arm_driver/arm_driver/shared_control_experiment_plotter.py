from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
import pandas as pd
import seaborn as sns
try:
    import yaml
except ImportError:  # pragma: no cover - 非 ROS 环境允许回退到默认参数
    yaml = None
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - 本地纯 Python 环境允许缺失
    get_package_share_directory = None

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    ROSBAG_IMPORT_ERROR: Exception | None = None
except ImportError as exc:  # pragma: no cover - 非 ROS 环境下允许只生成统计图
    rosbag2_py = None
    deserialize_message = None
    get_message = None
    ROSBAG_IMPORT_ERROR = exc

try:
    from .kinematics_solver import KinematicsSolver
except ImportError:  # pragma: no cover - 允许直接脚本方式运行
    from kinematics_solver import KinematicsSolver


sns.set_theme(style="whitegrid", context="paper", font_scale=1.2)

CONDITION_ORDER = ['base', 'shared']
CONDITION_TICKLABELS = ['Baseline', 'Shared Control']
TASK_ORDER = ['U', 'D', 'L', 'R', 'CW', 'CCW']
TASK_LABELS = {
    'U': 'Up',
    'D': 'Down',
    'L': 'Left',
    'R': 'Right',
    'CW': 'Circle CW',
    'CCW': 'Circle CCW',
}

CONTINUOUS_METRICS = {
    'max_overshoot_px': {
        'title': '{task}: Maximum Overshoot Distance (N={n_total})',
        'ylabel': 'Overshoot Distance (px)',
        'bar_color': 'lightcoral',
        'point_color': 'darkred',
    },
    'oob_duration_s': {
        'title': '{task}: Out-of-Bound Duration (N={n_total})',
        'ylabel': 'OOB Duration (s)',
        'bar_color': 'lightcoral',
        'point_color': 'darkred',
    },
    'recovery_time_s': {
        'title': '{task}: Recovery Time After Overshoot (N={n_total})',
        'ylabel': 'Recovery Time (s)',
        'bar_color': 'lightcoral',
        'point_color': 'darkred',
    },
    'completion_time_s': {
        'title': '{task}: Task Completion Time (N={n_total})',
        'ylabel': 'Completion Time (s)',
        'bar_color': 'lightblue',
        'point_color': 'darkblue',
    },
}

EVENT_METRICS = {
    'overshoot_event': {
        'title': '{task}: Overshoot Event Rate (N={n_total})',
        'legend_labels': ['Perfect (0 px)', 'Overshoot (>0 px)'],
    },
    'hard_return_event': {
        'title': '{task}: Hard Return Event Rate (N={n_total})',
        'legend_labels': ['No Hard Return', 'Hard Return Triggered'],
    },
    'slip_event': {
        'title': '{task}: Grip Slip Event Rate (N={n_total})',
        'legend_labels': ['No Slip', 'Slip Observed'],
    },
}

NUMERIC_COLUMNS = [
    'success',
    'completed_as_instructed',
    'repeat',
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
    'duration_s',
    'safe_radius_px',
    'free_radius_px',
    'hard_return_duration_s',
    'first_hard_return_time_s',
    'boundary_crossings',
    'slip_observed',
    'oscillation_observed',
    'hardware_fault',
]

RAW_TOPIC_NAMES = {
    '/laser_position',
    '/shared_control_feedback',
    '/joy_position',
    '/joint_states',
}

CONDITION_COLORS = {
    'base': '#1f77b4',
    'shared': '#ff7f0e',
}

DEFAULT_ARM_CONTROL_YAMLS = {
    'base': 'arm_control_shared_baseline.yaml',
    'shared': 'arm_control_shared_balanced.yaml',
}

ALPHA_INTERVENTION_THRESHOLD = 0.05
ARM_INTERVENTION_MIN_INTENT_M = 0.004
ARM_INTERVENTION_CORRECTION_M = 0.002
ARM_INTERVENTION_CONSECUTIVE_SAMPLES = 2
JOINT_STATE_HEIGHT_TOLERANCE_M = 0.015
_KINEMATICS_SOLVER = KinematicsSolver()


@dataclass(frozen=True)
class ControlParams:
    joystick_follow_scale_x_m: float = 0.016
    joystick_follow_scale_y_m: float = 0.016
    max_offset_x_m: float = 0.016
    max_offset_y_m: float = 0.016
    shared_height_m: float = 0.08
    grasp_x_m: float = 0.22
    grasp_y_m: float = 0.0


@dataclass
class RawTrialSeries:
    trial_id: str
    condition: str
    task: str
    repeat_label: str
    safe_radius_px: float
    free_radius_px: float
    laser_t_s: np.ndarray
    laser_x_px: np.ndarray
    laser_y_px: np.ndarray
    alpha_t_s: np.ndarray
    alpha: np.ndarray
    hard_return: np.ndarray
    desired_t_s: np.ndarray
    desired_x_m: np.ndarray
    desired_y_m: np.ndarray
    joint_t_s: np.ndarray
    actual_x_m: np.ndarray
    actual_y_m: np.ndarray
    alpha_intervention_t_s: float | None
    arm_intervention_t_s: float | None
    hard_return_t_s: float | None


def load_data(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    if df.empty:
        return df

    for column in NUMERIC_COLUMNS:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors='coerce')

    for column in ['trial_id', 'condition', 'task']:
        if column in df.columns:
            df[column] = df[column].fillna('').astype(str).str.strip()

    df = df[
        (df['trial_id'] != '')
        & (df['condition'] != '')
        & (df['task'] != '')
        & (df.get('max_laser_px').notna())
    ].copy()

    if df.empty:
        return df

    df = df[df['condition'].isin(CONDITION_ORDER)].copy()
    df['condition'] = pd.Categorical(df['condition'], categories=CONDITION_ORDER, ordered=True)
    df['condition_label'] = df['condition'].map({
        'base': 'Baseline',
        'shared': 'Shared Control',
    })
    df['task'] = pd.Categorical(df['task'], categories=TASK_ORDER, ordered=True)
    df['task_label'] = df['task'].map(TASK_LABELS)

    valid_mask = pd.Series(True, index=df.index)
    if 'success' in df.columns:
        valid_mask &= df['success'].fillna(1) > 0.5
    if 'completed_as_instructed' in df.columns:
        valid_mask &= df['completed_as_instructed'].fillna(1) > 0.5
    if 'hardware_fault' in df.columns:
        valid_mask &= df['hardware_fault'].fillna(0) < 0.5

    filtered_count = int((~valid_mask).sum())
    if filtered_count > 0:
        print(f'已按记录表过滤无效 trial: {filtered_count} 条')
        df = df[valid_mask].copy()
        if df.empty:
            return df

    df['overshoot_event'] = df.get('max_overshoot_px', 0).fillna(0) > 0
    df['hard_return_event'] = df.get('hard_return_duration_s', 0).fillna(0) > 0
    if 'slip_observed' in df.columns:
        df['slip_event'] = df['slip_observed'].fillna(0) > 0
    else:
        df['slip_event'] = False

    return df


def _format_repeat_label(value: object) -> str:
    if pd.isna(value):
        return ''
    try:
        return f'R{int(float(value)):02d}'
    except (TypeError, ValueError):
        return str(value)


def _numeric_or_default(value: object, default: float) -> float:
    if pd.isna(value):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _resolve_bag_path(row: pd.Series, bag_root: Path | None) -> Path | None:
    trial_id = str(row.get('trial_id', '')).strip()
    if bag_root is not None and trial_id:
        candidate = (bag_root / trial_id).expanduser().resolve()
        if candidate.exists():
            return candidate

    raw_bag_path = row.get('bag_path', '')
    if pd.isna(raw_bag_path):
        return None
    bag_path = Path(str(raw_bag_path).strip()).expanduser()
    if bag_path.exists():
        return bag_path.resolve()

    return None


def _default_arm_control_yaml_path(condition: str) -> Path | None:
    yaml_name = DEFAULT_ARM_CONTROL_YAMLS.get(condition)
    if not yaml_name:
        return None

    if get_package_share_directory is not None:
        try:
            share_dir = Path(get_package_share_directory('arm_driver'))
            candidate = share_dir / 'config' / yaml_name
            if candidate.exists():
                return candidate.resolve()
        except Exception:
            pass

    candidate = Path(__file__).resolve().parents[1] / 'config' / yaml_name
    if candidate.exists():
        return candidate.resolve()
    return None


def _load_control_params(row: pd.Series) -> ControlParams:
    default_params = ControlParams()
    raw_yaml = str(row.get('arm_control_yaml', '')).strip()
    yaml_path: Path | None = None
    if raw_yaml:
        candidate = Path(raw_yaml).expanduser()
        if candidate.exists():
            yaml_path = candidate.resolve()
    if yaml_path is None:
        yaml_path = _default_arm_control_yaml_path(str(row.get('condition', '')).strip())

    if yaml is None or yaml_path is None or not yaml_path.exists():
        return default_params

    try:
        raw_config = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
    except Exception:
        return default_params

    params = raw_config.get('arm_control_node', {}).get('ros__parameters', {})
    return ControlParams(
        joystick_follow_scale_x_m=float(
            params.get('joystick_follow_scale_x_m', default_params.joystick_follow_scale_x_m)
        ),
        joystick_follow_scale_y_m=float(
            params.get('joystick_follow_scale_y_m', default_params.joystick_follow_scale_y_m)
        ),
        max_offset_x_m=float(params.get('max_joystick_offset_x_m', default_params.max_offset_x_m)),
        max_offset_y_m=float(params.get('max_joystick_offset_y_m', default_params.max_offset_y_m)),
        shared_height_m=float(params.get('shared_control_height_m', default_params.shared_height_m)),
        grasp_x_m=default_params.grasp_x_m,
        grasp_y_m=default_params.grasp_y_m,
    )


def _load_raw_topic_types(reader: rosbag2_py.SequentialReader) -> dict[str, object]:
    if rosbag2_py is None or get_message is None:
        raise RuntimeError(f'rosbag2_py 不可用: {ROSBAG_IMPORT_ERROR}')
    topic_types: dict[str, object] = {}
    for topic_info in reader.get_all_topics_and_types():
        if topic_info.name in RAW_TOPIC_NAMES:
            topic_types[topic_info.name] = get_message(topic_info.type)
    return topic_types


def _interp_series(source_t: np.ndarray, source_values: np.ndarray, target_t: np.ndarray) -> np.ndarray:
    result = np.full(target_t.shape, np.nan, dtype=float)
    if source_t.size == 0 or source_values.size == 0 or target_t.size == 0:
        return result

    valid_mask = np.isfinite(source_t) & np.isfinite(source_values)
    if np.count_nonzero(valid_mask) == 0:
        return result

    time_values = source_t[valid_mask]
    series_values = source_values[valid_mask]
    if time_values.size == 1:
        coverage_mask = np.isclose(target_t, time_values[0]) | (target_t <= time_values[0])
        result[coverage_mask] = float(series_values[0])
        return result

    coverage_mask = (target_t >= time_values[0]) & (target_t <= time_values[-1])
    result[coverage_mask] = np.interp(target_t[coverage_mask], time_values, series_values)
    return result


def _first_threshold_crossing(
    times: np.ndarray,
    values: np.ndarray,
    threshold: float,
) -> float | None:
    if times.size == 0 or values.size == 0:
        return None
    valid_mask = np.isfinite(times) & np.isfinite(values)
    if np.count_nonzero(valid_mask) == 0:
        return None
    valid_times = times[valid_mask]
    valid_values = values[valid_mask]
    hit_indices = np.flatnonzero(valid_values >= threshold)
    if hit_indices.size == 0:
        return None
    return float(valid_times[int(hit_indices[0])])


def _first_boolean_run(
    times: np.ndarray,
    mask: np.ndarray,
    min_count: int = 1,
) -> float | None:
    if times.size == 0 or mask.size == 0:
        return None

    run_length = 0
    run_start_idx: int | None = None
    for idx, is_true in enumerate(mask):
        if bool(is_true):
            if run_length == 0:
                run_start_idx = idx
            run_length += 1
            if run_length >= max(1, min_count) and run_start_idx is not None:
                return float(times[run_start_idx])
        else:
            run_length = 0
            run_start_idx = None
    return None


def _compute_arm_intervention_time(
    joint_t_s: np.ndarray,
    actual_x_m: np.ndarray,
    actual_y_m: np.ndarray,
    desired_t_s: np.ndarray,
    desired_x_m: np.ndarray,
    desired_y_m: np.ndarray,
    alpha_t_s: np.ndarray,
    alpha_values: np.ndarray,
) -> float | None:
    if joint_t_s.size == 0 or desired_t_s.size == 0 or alpha_t_s.size == 0:
        return None

    desired_x_interp = _interp_series(desired_t_s, desired_x_m, joint_t_s)
    desired_y_interp = _interp_series(desired_t_s, desired_y_m, joint_t_s)
    alpha_interp = _interp_series(alpha_t_s, alpha_values, joint_t_s)
    desired_norm = np.hypot(desired_x_interp, desired_y_interp)
    correction_norm = np.hypot(actual_x_m - desired_x_interp, actual_y_m - desired_y_interp)

    valid_mask = (
        np.isfinite(desired_x_interp)
        & np.isfinite(desired_y_interp)
        & np.isfinite(alpha_interp)
        & np.isfinite(correction_norm)
    )
    intervention_mask = (
        valid_mask
        & (desired_norm >= ARM_INTERVENTION_MIN_INTENT_M)
        & (alpha_interp >= ALPHA_INTERVENTION_THRESHOLD)
        & (correction_norm >= ARM_INTERVENTION_CORRECTION_M)
    )
    return _first_boolean_run(
        joint_t_s,
        intervention_mask,
        min_count=ARM_INTERVENTION_CONSECUTIVE_SAMPLES,
    )


def _median_event_time(condition_series: list[RawTrialSeries], attr_name: str) -> float | None:
    event_values = []
    for series in condition_series:
        value = getattr(series, attr_name, None)
        if value is None:
            continue
        try:
            event_values.append(float(value))
        except (TypeError, ValueError):
            continue
    if not event_values:
        return None
    return float(np.median(np.asarray(event_values, dtype=float)))


def load_raw_trial_series(row: pd.Series, bag_root: Path | None) -> RawTrialSeries | None:
    if rosbag2_py is None or deserialize_message is None:
        raise RuntimeError(f'rosbag2_py 不可用: {ROSBAG_IMPORT_ERROR}')
    bag_dir = _resolve_bag_path(row, bag_root)
    if bag_dir is None or not bag_dir.exists():
        return None

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)
    topic_types = _load_raw_topic_types(reader)
    control_params = _load_control_params(row)

    bag_start_ns: int | None = None
    safe_radius_px = _numeric_or_default(row.get('safe_radius_px', 220.0), 220.0)
    free_radius_default = max(0.0, safe_radius_px - 60.0)
    free_radius_px = _numeric_or_default(row.get('free_radius_px', free_radius_default), free_radius_default)
    laser_t_s: list[float] = []
    laser_x_px: list[float] = []
    laser_y_px: list[float] = []
    alpha_t_s: list[float] = []
    alpha_values: list[float] = []
    hard_return_values: list[float] = []
    desired_t_s: list[float] = []
    desired_x_m: list[float] = []
    desired_y_m: list[float] = []
    joint_t_s: list[float] = []
    actual_x_m: list[float] = []
    actual_y_m: list[float] = []

    while reader.has_next():
        topic_name, serialized_data, timestamp_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = timestamp_ns
        if topic_name not in topic_types:
            continue

        elapsed_s = max(0.0, (timestamp_ns - bag_start_ns) / 1e9)
        msg = deserialize_message(serialized_data, topic_types[topic_name])
        values = list(getattr(msg, 'data', []))

        if topic_name == '/laser_position' and len(values) >= 7:
            safe_radius_px = float(values[6])
            free_radius_px = float(values[5])
            laser_t_s.append(elapsed_s)
            laser_x_px.append(float(values[0]) * safe_radius_px)
            laser_y_px.append(float(values[1]) * safe_radius_px)
        elif topic_name == '/joy_position' and len(values) >= 2:
            desired_t_s.append(elapsed_s)
            desired_x_m.append(
                float(
                    np.clip(
                        float(values[0]) * control_params.joystick_follow_scale_x_m,
                        -control_params.max_offset_x_m,
                        control_params.max_offset_x_m,
                    )
                )
            )
            desired_y_m.append(
                float(
                    np.clip(
                        float(values[1]) * control_params.joystick_follow_scale_y_m,
                        -control_params.max_offset_y_m,
                        control_params.max_offset_y_m,
                    )
                )
            )
        elif topic_name == '/joint_states':
            positions = list(getattr(msg, 'position', []))
            if len(positions) >= 4:
                actual_xyz, _ = _KINEMATICS_SOLVER.forward_kinematics(positions[:4])
                if abs(float(actual_xyz[2]) - control_params.shared_height_m) <= JOINT_STATE_HEIGHT_TOLERANCE_M:
                    joint_t_s.append(elapsed_s)
                    actual_x_m.append(float(actual_xyz[0] - control_params.grasp_x_m))
                    actual_y_m.append(float(actual_xyz[1] - control_params.grasp_y_m))
        elif topic_name == '/shared_control_feedback' and len(values) >= 4:
            alpha_t_s.append(elapsed_s)
            alpha_values.append(float(values[2]))
            hard_return_values.append(float(values[3]))

    if not laser_t_s and not alpha_t_s and not joint_t_s:
        return None

    alpha_t_s_arr = np.asarray(alpha_t_s, dtype=float)
    alpha_arr = np.asarray(alpha_values, dtype=float)
    hard_return_arr = np.asarray(hard_return_values, dtype=float)
    desired_t_s_arr = np.asarray(desired_t_s, dtype=float)
    desired_x_arr = np.asarray(desired_x_m, dtype=float)
    desired_y_arr = np.asarray(desired_y_m, dtype=float)
    joint_t_s_arr = np.asarray(joint_t_s, dtype=float)
    actual_x_arr = np.asarray(actual_x_m, dtype=float)
    actual_y_arr = np.asarray(actual_y_m, dtype=float)
    alpha_intervention_t_s = _first_threshold_crossing(
        alpha_t_s_arr,
        alpha_arr,
        ALPHA_INTERVENTION_THRESHOLD,
    )
    hard_return_t_s = _first_threshold_crossing(
        alpha_t_s_arr,
        hard_return_arr,
        0.5,
    )
    arm_intervention_t_s = _compute_arm_intervention_time(
        joint_t_s_arr,
        actual_x_arr,
        actual_y_arr,
        desired_t_s_arr,
        desired_x_arr,
        desired_y_arr,
        alpha_t_s_arr,
        alpha_arr,
    )

    return RawTrialSeries(
        trial_id=str(row.get('trial_id', '')).strip(),
        condition=str(row.get('condition', '')).strip(),
        task=str(row.get('task', '')).strip(),
        repeat_label=_format_repeat_label(row.get('repeat')),
        safe_radius_px=safe_radius_px,
        free_radius_px=free_radius_px,
        laser_t_s=np.asarray(laser_t_s, dtype=float),
        laser_x_px=np.asarray(laser_x_px, dtype=float),
        laser_y_px=np.asarray(laser_y_px, dtype=float),
        alpha_t_s=alpha_t_s_arr,
        alpha=alpha_arr,
        hard_return=hard_return_arr,
        desired_t_s=desired_t_s_arr,
        desired_x_m=desired_x_arr,
        desired_y_m=desired_y_arr,
        joint_t_s=joint_t_s_arr,
        actual_x_m=actual_x_arr,
        actual_y_m=actual_y_arr,
        alpha_intervention_t_s=alpha_intervention_t_s,
        arm_intervention_t_s=arm_intervention_t_s,
        hard_return_t_s=hard_return_t_s,
    )


def build_raw_series_cache(df: pd.DataFrame, bag_root: Path | None) -> dict[str, RawTrialSeries]:
    cache: dict[str, RawTrialSeries] = {}
    for _, row in df.sort_values(['task', 'condition', 'repeat']).iterrows():
        trial_id = str(row.get('trial_id', '')).strip()
        if not trial_id or trial_id in cache:
            continue
        try:
            series = load_raw_trial_series(row, bag_root)
        except Exception as exc:  # pragma: no cover - bag 异常时允许跳过
            print(f'跳过原始图读取失败的 trial: {trial_id} ({exc})')
            continue
        if series is not None:
            cache[trial_id] = series
    return cache


def summarize_values(series: pd.Series) -> dict[str, object]:
    clean = series.dropna()
    if clean.empty:
        return {
            'n': 0,
            'mean': '',
            'std': '',
            'median': '',
            'min': '',
            'max': '',
        }
    return {
        'n': int(clean.shape[0]),
        'mean': round(float(clean.mean()), 4),
        'std': round(float(clean.std(ddof=1)) if clean.shape[0] >= 2 else 0.0, 4),
        'median': round(float(clean.median()), 4),
        'min': round(float(clean.min()), 4),
        'max': round(float(clean.max()), 4),
    }


def write_summary_tables(df: pd.DataFrame, tables_dir: Path) -> list[Path]:
    tables_dir.mkdir(parents=True, exist_ok=True)
    created_paths: list[Path] = []

    condition_rows: list[dict[str, object]] = []
    for condition in CONDITION_ORDER:
        condition_df = df[df['condition'] == condition]
        if condition_df.empty:
            continue
        for metric_key in CONTINUOUS_METRICS:
            condition_rows.append({
                'condition': CONDITION_TICKLABELS[CONDITION_ORDER.index(condition)],
                'metric': metric_key,
                **summarize_values(condition_df[metric_key]),
            })
    condition_path = tables_dir / 'summary_by_condition.csv'
    pd.DataFrame(condition_rows).to_csv(condition_path, index=False)
    created_paths.append(condition_path)

    task_rows: list[dict[str, object]] = []
    for task in TASK_ORDER:
        task_df = df[df['task'] == task]
        if task_df.empty:
            continue
        for condition in CONDITION_ORDER:
            group_df = task_df[task_df['condition'] == condition]
            if group_df.empty:
                continue
            for metric_key in CONTINUOUS_METRICS:
                task_rows.append({
                    'condition': CONDITION_TICKLABELS[CONDITION_ORDER.index(condition)],
                    'task': TASK_LABELS.get(task, task),
                    'metric': metric_key,
                    **summarize_values(group_df[metric_key]),
                })
    task_path = tables_dir / 'summary_by_condition_task.csv'
    pd.DataFrame(task_rows).to_csv(task_path, index=False)
    created_paths.append(task_path)

    event_rows: list[dict[str, object]] = []
    for task in TASK_ORDER:
        task_df = df[df['task'] == task]
        if task_df.empty:
            continue
        for condition in CONDITION_ORDER:
            group_df = task_df[task_df['condition'] == condition]
            if group_df.empty:
                continue
            n = int(group_df.shape[0])
            event_rows.append({
                'condition': CONDITION_TICKLABELS[CONDITION_ORDER.index(condition)],
                'task': TASK_LABELS.get(task, task),
                'n': n,
                'overshoot_trials': int(group_df['overshoot_event'].sum()),
                'overshoot_rate': round(float(group_df['overshoot_event'].mean()), 4),
                'hard_return_trials': int(group_df['hard_return_event'].sum()),
                'hard_return_rate': round(float(group_df['hard_return_event'].mean()), 4),
                'slip_trials': int(group_df['slip_event'].sum()),
                'slip_rate': round(float(group_df['slip_event'].mean()), 4),
            })
    event_path = tables_dir / 'summary_event_rates.csv'
    pd.DataFrame(event_rows).to_csv(event_path, index=False)
    created_paths.append(event_path)

    return created_paths


def _apply_barplot_styling(ax: plt.Axes) -> None:
    for patch in ax.patches:
        patch.set_alpha(0.7)
        patch.set_edgecolor('black')
        patch.set_linewidth(1.0)
    for line in ax.lines:
        line.set_color('black')
        line.set_linewidth(1.0)


def _condition_counts(df: pd.DataFrame) -> pd.Series:
    return df.groupby('condition').size().reindex(CONDITION_ORDER, fill_value=0)


def _format_valid_note(task_df: pd.DataFrame, plot_df: pd.DataFrame) -> str | None:
    total_counts = task_df.groupby('condition').size().reindex(CONDITION_ORDER, fill_value=0)
    valid_counts = plot_df.groupby('condition').size().reindex(CONDITION_ORDER, fill_value=0)
    if valid_counts.equals(total_counts):
        return None

    return (
        'Valid n: '
        f'Baseline={int(valid_counts["base"])}/{int(total_counts["base"])}, '
        f'Shared={int(valid_counts["shared"])}/{int(total_counts["shared"])}'
    )


def plot_continuous_metric(task_df: pd.DataFrame, task: str, metric_key: str, output_path: Path) -> bool:
    metric_cfg = CONTINUOUS_METRICS[metric_key]
    plot_df = task_df[task_df[metric_key].notna()].copy()
    if plot_df.empty:
        return False

    n_total = int(task_df.shape[0])
    valid_note = _format_valid_note(task_df, plot_df)

    fig, ax = plt.subplots(figsize=(6, 5))
    sns.barplot(
        data=plot_df,
        x='condition',
        y=metric_key,
        order=CONDITION_ORDER,
        estimator=np.mean,
        ci='sd',
        capsize=0.1,
        color=metric_cfg['bar_color'],
        edgecolor='black',
        ax=ax,
    )
    _apply_barplot_styling(ax)

    sns.stripplot(
        data=plot_df,
        x='condition',
        y=metric_key,
        order=CONDITION_ORDER,
        color=metric_cfg['point_color'],
        size=8,
        jitter=True,
        alpha=0.8,
        ax=ax,
    )

    ax.set_xlabel('')
    ax.set_ylabel(metric_cfg['ylabel'])
    ax.set_xticklabels(CONDITION_TICKLABELS)
    ax.set_title(
        metric_cfg['title'].format(
            task=TASK_LABELS.get(task, task),
            n_total=n_total,
        ),
        pad=15,
    )
    if valid_note:
        ax.text(
            0.02,
            0.98,
            valid_note,
            transform=ax.transAxes,
            ha='left',
            va='top',
            fontsize=10,
            color='dimgray',
        )
    if metric_key in {'max_overshoot_px', 'oob_duration_s', 'recovery_time_s', 'completion_time_s'}:
        ymax = float(plot_df[metric_key].max()) if not plot_df[metric_key].empty else 0.0
        ax.set_ylim(0.0, max(1.0, ymax * 1.15))

    sns.despine(ax=ax, top=True, right=True)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    return True


def plot_event_rate(task_df: pd.DataFrame, task: str, event_key: str, output_path: Path) -> bool:
    if event_key not in task_df.columns:
        return False

    plot_df = task_df.copy()
    if plot_df.empty:
        return False

    grouped = plot_df.groupby('condition')[event_key].agg(['sum', 'count']).reindex(CONDITION_ORDER)
    grouped = grouped.fillna(0)
    grouped['good_pct'] = (grouped['count'] - grouped['sum']) / grouped['count'].replace(0, pd.NA) * 100.0
    grouped['bad_pct'] = grouped['sum'] / grouped['count'].replace(0, pd.NA) * 100.0
    grouped = grouped.fillna(0.0)

    fig, ax = plt.subplots(figsize=(5, 5))
    x_positions = range(len(CONDITION_ORDER))
    ax.bar(
        list(x_positions),
        grouped['good_pct'].tolist(),
        color='#2ca02c',
        alpha=0.8,
        edgecolor='black',
        linewidth=1.0,
    )
    ax.bar(
        list(x_positions),
        grouped['bad_pct'].tolist(),
        bottom=grouped['good_pct'].tolist(),
        color='#d62728',
        alpha=0.8,
        edgecolor='black',
        linewidth=1.0,
    )

    for x_pos, (_, row) in zip(x_positions, grouped.iterrows()):
        total = int(row['count'])
        bad = int(row['sum'])
        ax.text(
            x_pos,
            min(98.0, float(row['bad_pct']) + 3.0),
            f'{bad}/{total}',
            ha='center',
            va='bottom',
            fontsize=10,
        )

    legend_cfg = EVENT_METRICS[event_key]
    handles = [
        Patch(facecolor='#2ca02c', edgecolor='black', alpha=0.8, label=legend_cfg['legend_labels'][0]),
        Patch(facecolor='#d62728', edgecolor='black', alpha=0.8, label=legend_cfg['legend_labels'][1]),
    ]
    ax.legend(handles=handles, loc='center left', bbox_to_anchor=(1.0, 0.5), frameon=False)
    ax.set_ylim(0, 100)
    ax.set_ylabel('Percentage of Trials (%)')
    ax.set_xlabel('')
    ax.set_xticks(list(x_positions))
    ax.set_xticklabels(CONDITION_TICKLABELS)
    ax.set_title(
        legend_cfg['title'].format(
            task=TASK_LABELS.get(task, task),
            n_total=int(task_df.shape[0]),
        ),
        pad=15,
    )

    sns.despine(ax=ax, top=True, right=True)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    return True


def _valid_raw_series(task_df: pd.DataFrame, raw_cache: dict[str, RawTrialSeries], condition: str) -> list[RawTrialSeries]:
    series_list: list[RawTrialSeries] = []
    condition_df = task_df[task_df['condition'] == condition].sort_values('repeat')
    for _, row in condition_df.iterrows():
        trial_id = str(row.get('trial_id', '')).strip()
        series = raw_cache.get(trial_id)
        if series is None:
            continue
        series_list.append(series)
    return series_list


def _mean_alpha_trace(condition_series: list[RawTrialSeries]) -> tuple[np.ndarray, np.ndarray] | None:
    durations = [float(series.alpha_t_s[-1]) for series in condition_series if series.alpha_t_s.size > 0]
    if not durations:
        return None
    max_duration = max(durations)
    if max_duration <= 0.0:
        return None

    time_grid = np.linspace(0.0, max_duration, 400)
    interpolated = np.full((len(condition_series), time_grid.size), np.nan, dtype=float)
    for idx, series in enumerate(condition_series):
        if series.alpha_t_s.size == 0:
            continue
        if series.alpha_t_s.size == 1:
            interpolated[idx, time_grid <= series.alpha_t_s[-1]] = float(series.alpha[0])
            continue
        valid_mask = np.isfinite(series.alpha_t_s) & np.isfinite(series.alpha)
        if np.count_nonzero(valid_mask) < 2:
            continue
        time_values = series.alpha_t_s[valid_mask]
        alpha_values = series.alpha[valid_mask]
        coverage_mask = time_grid <= time_values[-1]
        interpolated[idx, coverage_mask] = np.interp(time_grid[coverage_mask], time_values, alpha_values)

    valid_points = np.any(np.isfinite(interpolated), axis=0)
    if not np.any(valid_points):
        return None
    mean_trace = np.full(time_grid.shape, np.nan, dtype=float)
    mean_trace[valid_points] = np.nanmean(interpolated[:, valid_points], axis=0)
    return time_grid, mean_trace


def plot_alpha_timeseries(
    task_df: pd.DataFrame,
    task: str,
    raw_cache: dict[str, RawTrialSeries],
    output_path: Path,
) -> bool:
    task_series = [
        raw_cache[str(row.get('trial_id', '')).strip()]
        for _, row in task_df.iterrows()
        if str(row.get('trial_id', '')).strip() in raw_cache
        and raw_cache[str(row.get('trial_id', '')).strip()].alpha_t_s.size > 0
    ]
    if not task_series:
        return False

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), sharey=True)
    max_duration = max(
        float(series.alpha_t_s[-1])
        for series in task_series
        if series.alpha_t_s.size > 0
    )

    for ax, condition in zip(axes, CONDITION_ORDER):
        condition_series = _valid_raw_series(task_df, raw_cache, condition)
        condition_series = [series for series in condition_series if series.alpha_t_s.size > 0]
        condition_color = CONDITION_COLORS[condition]
        mean_trace: tuple[np.ndarray, np.ndarray] | None = None

        for series in condition_series:
            ax.plot(
                series.alpha_t_s,
                series.alpha,
                color=condition_color,
                alpha=0.25,
                linewidth=1.1,
            )

        mean_trace = _mean_alpha_trace(condition_series)
        if mean_trace is not None:
            time_grid, mean_values = mean_trace
            ax.plot(
                time_grid,
                mean_values,
                color=condition_color,
                linewidth=2.6,
                alpha=0.95,
                label='Mean alpha',
            )

        arm_marker_t_s = _median_event_time(condition_series, 'arm_intervention_t_s')
        if arm_marker_t_s is not None:
            ax.axvline(
                arm_marker_t_s,
                color='black',
                linestyle='--',
                linewidth=1.5,
                alpha=0.85,
                label='Arm intervention (median)',
            )
            if mean_trace is not None:
                marker_y = _interp_series(
                    mean_trace[0],
                    mean_trace[1],
                    np.asarray([arm_marker_t_s], dtype=float),
                )[0]
                if np.isfinite(marker_y):
                    ax.scatter(
                        [arm_marker_t_s],
                        [marker_y],
                        color='black',
                        marker='D',
                        s=32,
                        zorder=6,
                    )

        ax.set_xlim(0.0, max(1.0, max_duration * 1.02))
        ax.set_ylim(-0.02, 1.02)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Alpha')
        ax.set_title(f'{CONDITION_TICKLABELS[CONDITION_ORDER.index(condition)]} (n={len(condition_series)})')
        if mean_trace is not None or arm_marker_t_s is not None:
            ax.legend(loc='upper right', framealpha=0.9, fontsize=8.5)

    fig.suptitle(f'{TASK_LABELS.get(task, task)}: Alpha Time Histories', y=1.02)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    return True


def generate_task_figures(df: pd.DataFrame, figures_dir: Path) -> list[Path]:
    created_paths: list[Path] = []
    continuous_dir = figures_dir / 'continuous'
    event_dir = figures_dir / 'events'

    for task in TASK_ORDER:
        task_df = df[df['task'] == task].copy()
        if task_df.empty:
            continue

        for metric_key in CONTINUOUS_METRICS:
            output_path = continuous_dir / f'task_{task}_{metric_key}.png'
            if plot_continuous_metric(task_df, task, metric_key, output_path):
                created_paths.append(output_path)

        for event_key in EVENT_METRICS:
            output_path = event_dir / f'task_{task}_{event_key}.png'
            if plot_event_rate(task_df, task, event_key, output_path):
                created_paths.append(output_path)

    return created_paths


def generate_raw_task_figures(
    df: pd.DataFrame,
    figures_dir: Path,
    bag_root: Path | None,
) -> list[Path]:
    created_paths: list[Path] = []
    if rosbag2_py is None:
        print(f'跳过 alpha 时序图生成：rosbag2_py 不可用 ({ROSBAG_IMPORT_ERROR})')
        return created_paths
    raw_cache = build_raw_series_cache(df, bag_root)
    if not raw_cache:
        return created_paths

    alpha_dir = figures_dir / 'alpha_timeseries'
    for task in TASK_ORDER:
        task_df = df[df['task'] == task].copy()
        if task_df.empty:
            continue

        alpha_path = alpha_dir / f'task_{task}_alpha_timeseries.png'
        if plot_alpha_timeseries(task_df, task, raw_cache, alpha_path):
            created_paths.append(alpha_path)

    return created_paths


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='从共享控制实验 CSV 生成论文风格的汇总表和分任务图表。',
    )
    parser.add_argument(
        'analysis_csv',
        help='由 shared_control_experiment_analyzer 输出的 analysis CSV 或 merged CSV。',
    )
    parser.add_argument(
        '--output-dir',
        default='shared_control_experiment_plots',
        help='输出目录，内部会自动创建 tables/ 和 figures/。',
    )
    parser.add_argument(
        '--bag-root',
        default='',
        help='可选：原始 rosbag 根目录。若提供，则优先按 {bag_root}/{trial_id} 读取原始 bag 用于 alpha 时序图。',
    )
    return parser


def main(args: list[str] | None = None) -> None:
    parser = build_arg_parser()
    parsed = parser.parse_args(args=args)

    analysis_csv = Path(parsed.analysis_csv).expanduser().resolve()
    df = load_data(analysis_csv)
    if df.empty:
        raise SystemExit(f'CSV 中没有可用的实验数据: {analysis_csv}')

    output_dir = Path(parsed.output_dir).expanduser().resolve()
    tables_dir = output_dir / 'tables'
    figures_dir = output_dir / 'figures'
    bag_root = Path(parsed.bag_root).expanduser().resolve() if parsed.bag_root else None

    table_paths = write_summary_tables(df, tables_dir)
    figure_paths = generate_task_figures(df, figures_dir)
    raw_figure_paths = generate_raw_task_figures(df, figures_dir, bag_root)

    for table_path in table_paths:
        print(f'已输出汇总表: {table_path}')
    for figure_path in figure_paths:
        print(f'已输出统计图: {figure_path}')
    for figure_path in raw_figure_paths:
        print(f'已输出 alpha 时序图: {figure_path}')


if __name__ == '__main__':
    main()
