from __future__ import annotations

import argparse
from pathlib import Path
import os
import signal
import subprocess
import sys
import time


DEFAULT_BAG_ROOT = Path('/home/pq/arm_project_ws/experiment_outputs/bag')
DEFAULT_TOPICS = [
    '/joy_position',
    '/virtual_fixture_force',
    '/laser_position',
    '/shared_control_feedback',
    '/joint_states',
    '/gripper_contact',
    '/gripper_overpressure',
    '/gripper_sensor_fault',
    '/force_sensor',
]
VALID_TASKS = ['U', 'D', 'L', 'R', 'CW', 'CCW']
VALID_CONDITIONS = ['base', 'shared']


def build_trial_ids(
    condition: str,
    tasks: list[str],
    repeat_start: int,
    repeat_end: int,
    repeat_major: bool,
) -> list[str]:
    repeats = list(range(repeat_start, repeat_end + 1))
    trial_ids: list[str] = []
    if repeat_major:
        for repeat in repeats:
            for task in tasks:
                trial_ids.append(f'C{condition}_T{task}_R{repeat:02d}')
    else:
        for task in tasks:
            for repeat in repeats:
                trial_ids.append(f'C{condition}_T{task}_R{repeat:02d}')
    return trial_ids


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='在同一个终端内顺序录制多条实验 rosbag，按回车开始/停止每条 trial。',
    )
    parser.add_argument(
        'trial_ids',
        nargs='*',
        help='可直接给出 TrialID 列表，例如 Cbase_TU_R06 Cbase_TU_R07 Cbase_TU_R08 Cbase_TU_R09 Cbase_TU_R10',
    )
    parser.add_argument(
        '--condition',
        choices=VALID_CONDITIONS,
        help='若不直接给出 trial_ids，可用 condition + tasks + repeat 范围自动生成。',
    )
    parser.add_argument(
        '--tasks',
        nargs='+',
        choices=VALID_TASKS,
        help='自动生成 TrialID 时使用，例如 --tasks U D',
    )
    parser.add_argument(
        '--repeat-start',
        type=int,
        help='自动生成 TrialID 时的起始 repeat，例如 6',
    )
    parser.add_argument(
        '--repeat-end',
        type=int,
        help='自动生成 TrialID 时的结束 repeat，例如 10',
    )
    parser.add_argument(
        '--repeat-major',
        action='store_true',
        help='自动生成时按 repeat 交替任务排序，例如 U6 D6 U7 D7。',
    )
    parser.add_argument(
        '--bag-root',
        default=str(DEFAULT_BAG_ROOT),
        help='rosbag 输出根目录。',
    )
    parser.add_argument(
        '--startup-wait-sec',
        type=float,
        default=1.5,
        help='启动 ros2 bag record 后等待多少秒再提示开始动作，默认 1.5 秒。',
    )
    parser.add_argument(
        '--print-only',
        action='store_true',
        help='只打印将要录制的 TrialID，不真正启动 rosbag。',
    )
    return parser


def resolve_trial_ids(parsed: argparse.Namespace) -> list[str]:
    raw_trial_ids = [trial_id.strip() for trial_id in parsed.trial_ids if trial_id.strip()]
    if raw_trial_ids:
        return raw_trial_ids

    if not parsed.condition or not parsed.tasks or parsed.repeat_start is None or parsed.repeat_end is None:
        raise SystemExit(
            '请直接提供 trial_ids，或使用 --condition + --tasks + --repeat-start + --repeat-end 自动生成。'
        )
    if parsed.repeat_end < parsed.repeat_start:
        raise SystemExit('--repeat-end 不能小于 --repeat-start。')

    return build_trial_ids(
        condition=parsed.condition,
        tasks=parsed.tasks,
        repeat_start=parsed.repeat_start,
        repeat_end=parsed.repeat_end,
        repeat_major=parsed.repeat_major,
    )


def stop_recording(proc: subprocess.Popen[bytes] | None, timeout_sec: float = 10.0) -> None:
    if proc is None or proc.poll() is not None:
        return

    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            proc.wait(timeout=1.0)


def prompt_user(message: str) -> str:
    try:
        return input(message).strip().lower()
    except EOFError:
        return 'q'


def main(args: list[str] | None = None) -> None:
    parser = build_arg_parser()
    parsed = parser.parse_args(args=args)

    trial_ids = resolve_trial_ids(parsed)
    bag_root = Path(parsed.bag_root).expanduser().resolve()

    print('本次顺序录制的 TrialID：')
    for index, trial_id in enumerate(trial_ids, start=1):
        print(f'  {index:02d}. {trial_id}')

    if parsed.print_only:
        return

    bag_root.mkdir(parents=True, exist_ok=True)

    active_proc: subprocess.Popen[bytes] | None = None
    try:
        for index, trial_id in enumerate(trial_ids, start=1):
            bag_dir = bag_root / trial_id
            if bag_dir.exists():
                print(f'\n[{index}/{len(trial_ids)}] 跳过：bag 目录已存在 -> {bag_dir}')
                continue

            command = ['ros2', 'bag', 'record', '-o', str(bag_dir), *DEFAULT_TOPICS]
            print(f'\n[{index}/{len(trial_ids)}] 下一条：{trial_id}')
            print(f'输出目录：{bag_dir}')
            user_choice = prompt_user('按 Enter 开始录制；输入 s 跳过；输入 q 退出：')
            if user_choice == 'q':
                break
            if user_choice == 's':
                continue

            active_proc = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            time.sleep(max(0.0, parsed.startup_wait_sec))
            print(f'已开始录制 {trial_id}。完成动作后回到本终端。')

            user_choice = prompt_user('按 Enter 停止并保存本条；输入 q 停止后退出：')
            stop_recording(active_proc)
            active_proc = None

            print(f'已保存：{bag_dir}')
            print(f'提示：现在可以在 Pygame 里按 S 保存截图，再运行 shared_control_trial_logger 记录备注。')

            if user_choice == 'q':
                break

    except KeyboardInterrupt:
        print('\n检测到 Ctrl+C，正在停止当前 rosbag 录制...')
        stop_recording(active_proc)
        raise SystemExit(130)
    finally:
        stop_recording(active_proc)


if __name__ == '__main__':
    main()
