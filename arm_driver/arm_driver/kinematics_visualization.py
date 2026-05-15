"""
机械臂运动学可视化脚本 (双语终极版 + 边界对齐)
用途：毕业论文第三章 — 运动学建模与工作空间分析
优化内容：
1. 自动生成中文版和英文版配图
2. z=0.08m 共享平面边界与ROS实际控制节点严格对齐 (16x16mm)
3. 向量化提速与 3D 物理比例真实化
"""

import math
import argparse
from pathlib import Path
from typing import Tuple, Dict

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 全局字体与绘图质量设置
plt.rcParams['font.sans-serif'] = ['Arial Unicode MS', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.autolayout'] = True
plt.rcParams['axes.grid'] = True

# ─────────────────────────────────────────────────────────────────────────────
# 机械臂参数
# ─────────────────────────────────────────────────────────────────────────────
D1     = 0.060   
L1     = 0.0814  
L2     = 0.0814  
L_END  = 0.140   

Q1_LIM     = math.radians(135)
Q2_RAD_MIN = math.radians(-(240 - 90))  
Q2_RAD_MAX = math.radians(90 - (-60))   

# ─────────────────────────────────────────────────────────────────────────────
# 运动学核心函数 (向量化支持)
# ─────────────────────────────────────────────────────────────────────────────
def forward_kinematics(q1, q2, q3, q4) -> Tuple:
    phi = q2 + q3 + q4
    r2  = L1 * np.cos(q2) + L2 * np.cos(q2 + q3) + L_END * np.cos(phi)
    x   = r2 * np.cos(q1)
    y   = r2 * np.sin(q1)
    z   = D1 + L1 * np.sin(q2) + L2 * np.sin(q2 + q3) + L_END * np.sin(phi)
    return x, y, z, phi

def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def get_all_frames(q1, q2, q3, q4) -> Tuple[list, list]:
    T0 = np.eye(4)
    T1 = T0 @ dh_matrix(q1, D1,    0,     math.pi/2)
    T2 = T1 @ dh_matrix(q2, 0,     L1,    0)
    T3 = T2 @ dh_matrix(q3, 0,     L2,    0)
    T4 = T3 @ dh_matrix(q4, 0,     L_END, 0)
    
    origins = [T[:3, 3] for T in (T0, T1, T2, T3, T4)]
    axes_scale = 0.025
    frames = []
    for T in (T0, T1, T2, T3, T4):
        frames.append({
            'o': T[:3, 3],
            'x': T[:3, 0] * axes_scale,
            'y': T[:3, 1] * axes_scale,
            'z': T[:3, 2] * axes_scale,
        })
    return origins, frames

# ─────────────────────────────────────────────────────────────────────────────
# 绘图模块 (支持双语 lang='cn' 或 'en')
# ─────────────────────────────────────────────────────────────────────────────
def set_axes_equal_3d(ax):
    x_limits, y_limits, z_limits = ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    max_range = max([x_range, y_range, z_range])
    ax.set_box_aspect((x_range/max_range, y_range/max_range, z_range/max_range))


def plot_arm_with_frames(q1, q2, q3, q4, lang='cn', title=None, ax=None):
    if ax is None:
        fig = plt.figure(figsize=(8, 7), dpi=120)
        ax = fig.add_subplot(111, projection='3d')

    origins, frames = get_all_frames(q1, q2, q3, q4)
    pts = np.array(origins)

    label_link = '机械臂连杆' if lang == 'cn' else 'Arm Links'
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'o-', color='#2c3e50', linewidth=3.5, markersize=8, label=label_link, zorder=5)

    colors = {'x': '#e74c3c', 'y': '#2ecc71', 'z': '#3498db'}
    added = set()
    for f in frames:
        o = f['o']
        for axis in ('x', 'y', 'z'):
            label = (f"{axis.upper()}轴" if lang == 'cn' else f"{axis.upper()}-Axis") if axis not in added else ""
            ax.quiver(*o, *f[axis], color=colors[axis], linewidth=1.5, label=label)
            added.add(axis)

    joint_names = ['O₀(世界)', 'O₁(底座)', 'O₂(肩)', 'O₃(肘)', 'O₄(末端)'] if lang == 'cn' else \
                  ['O₀(World)', 'O₁(Base)', 'O₂(Shoulder)', 'O₃(Elbow)', 'O₄(End)']
    for p, name in zip(pts, joint_names):
        ax.text(p[0], p[1], p[2]+0.015, name, fontsize=9, color='#34495e')

    xx, yy = np.meshgrid([-0.05, 0.40], [-0.25, 0.25])
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.1, color='gray')
    ax.plot_surface(xx, yy, np.full_like(xx, 0.08), alpha=0.15, color='#3498db')
    
    plane_text = 'z=0.08m 共享平面' if lang == 'cn' else 'z=0.08m Shared Plane'
    ax.text(0.30, 0.0, 0.082, plane_text, fontsize=9, color='#2980b9')

    if title is None:
        title = '机械臂连杆结构与各关节坐标系 (示例)' if lang == 'cn' else 'Arm Kinematics & Joint Frames (Example)'
        
    ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title=title)
    ax.set_xlim(-0.05, 0.40); ax.set_ylim(-0.25, 0.25); ax.set_zlim(0, 0.35)
    ax.view_init(elev=25, azim=45)
    set_axes_equal_3d(ax)

    handles, labs = ax.get_legend_handles_labels()
    ul = dict(zip(labs, handles))
    ax.legend(ul.values(), ul.keys(), loc='upper left', framealpha=0.9)
    return ax


def compute_workspace_vectorized(n_samples=120000):
    rng = np.random.default_rng(42)
    q1s = rng.uniform(-Q1_LIM, Q1_LIM, n_samples)
    q2s = rng.uniform(Q2_RAD_MIN, Q2_RAD_MAX, n_samples)
    q3s = rng.uniform(-math.radians(150), math.radians(150), n_samples)
    q4s = rng.uniform(-math.radians(90), math.radians(90), n_samples)
    x, y, z, _ = forward_kinematics(q1s, q2s, q3s, q4s)
    valid = z >= 0.0
    return x[valid], y[valid], z[valid]


def plot_workspace(xs, ys, zs, lang='cn', ax=None):
    if ax is None:
        fig = plt.figure(figsize=(9, 7), dpi=120)
        ax = fig.add_subplot(111, projection='3d')

    sc = ax.scatter(xs, ys, zs, c=zs, cmap='plasma', s=0.2, alpha=0.4, edgecolors='none')
    cbar = plt.colorbar(sc, ax=ax, shrink=0.5, pad=0.1)
    
    cbar_label = 'Z 高度 (m)' if lang == 'cn' else 'Z Height (m)'
    cbar.set_label(cbar_label, rotation=270, labelpad=15)

    key_pts = {
        'Home (0.20,0,0.18)':   (0.20, 0.0, 0.18),
        'Transit (0.22,0,0.18)':(0.22, 0.0, 0.18),
        'Hover (0.22,0,0.10)':  (0.22, 0.0, 0.10),
        'Grasp (0.22,0,0.08)':  (0.22, 0.0, 0.08),
    }
    for label, (px, py, pz) in key_pts.items():
        ax.scatter(px, py, pz, color='#e74c3c', s=50, edgecolors='white', zorder=10)
        ax.text(px, py, pz+0.015, label, fontsize=8, color='#c0392b')

    title = '机械臂三维工作空间 (蒙特卡洛采样)' if lang == 'cn' else '3D Workspace (Monte Carlo Sampling)'
    ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title=title)
    ax.view_init(elev=30, azim=60)
    set_axes_equal_3d(ax)
    return ax


def plot_workspace_slice(xs, ys, zs, z_target=0.08, tolerance=0.005, lang='cn'):
    mask = np.abs(zs - z_target) < tolerance
    x_sl, y_sl = xs[mask], ys[mask]

    fig, ax = plt.subplots(figsize=(7, 7), dpi=120)
    
    label_pts = f'可达点 (z={z_target}±{tolerance}m)' if lang == 'cn' else f'Reachable Points (z={z_target}±{tolerance}m)'
    ax.scatter(x_sl, y_sl, s=2, alpha=0.3, color='#3498db', label=label_pts)

    # 与当前正式实验 YAML 对齐，实际共享边界为 32 mm x 32 mm (偏移量 ±16 mm)
    from matplotlib.patches import Rectangle
    label_rect = '共享控制区 [±16mm]' if lang == 'cn' else 'Shared Control Region [±16mm]'
    rect = Rectangle((0.204, -0.016), 0.032, 0.032, linewidth=2, edgecolor='#e74c3c', 
                     facecolor='#e74c3c', alpha=0.2, label=label_rect)
    ax.add_patch(rect)
    
    label_center = '抓取中心 (0.22, 0)' if lang == 'cn' else 'Grasp Center (0.22, 0)'
    ax.plot(0.22, 0.0, 'r*', markersize=12, label=label_center)
    
    label_base = '底座原点' if lang == 'cn' else 'Base Origin'
    ax.plot(0.0, 0.0, 'ks', markersize=8, label=label_base)

    title = f'z={z_target}m 平面可达域截面' if lang == 'cn' else f'Reachable Workspace Cross-section at z={z_target}m'
    ax.set(xlabel='X (m)', ylabel='Y (m)', title=title)
    ax.set_aspect('equal')
    ax.legend(fontsize=10, framealpha=0.9)
    return fig, ax


def solve_four_configs(target_pos, target_phi=0.0, lang='cn'):
    x, y, z = target_pos
    results = {}
    q1_front = math.atan2(y, x)
    r_front  = math.sqrt(x**2 + y**2)
    q1_back  = q1_front + math.pi if q1_front <= 0 else q1_front - math.pi
    r_back   = -r_front
    phi_back = math.pi - target_phi

    if lang == 'cn':
        c_names = ['正向-肘上', '正向-肘下', '背向-肘上', '背向-肘下']
    else:
        c_names = ['FWD-Elbow Up', 'FWD-Elbow Down', 'BWD-Elbow Up', 'BWD-Elbow Down']

    configs = [
        (c_names[0], q1_front, r_front, target_phi, 'up'),
        (c_names[1], q1_front, r_front, target_phi, 'down'),
        (c_names[2], q1_back,  r_back,  phi_back,   'up'),
        (c_names[3], q1_back,  r_back,  phi_back,   'down'),
    ]

    for name, q1, r, phi_local, elbow in configs:
        r_w = r - L_END * math.cos(phi_local)
        z_w = z - L_END * math.sin(phi_local)
        dz  = z_w - D1
        D   = math.sqrt(r_w**2 + dz**2)

        if D > (L1 + L2) or D < abs(L1 - L2):
            results[name] = None
            continue

        cos_q3 = max(min((D**2 - L1**2 - L2**2) / (2 * L1 * L2), 1.0), -1.0)
        q3 = -math.acos(cos_q3) if elbow == 'up' else math.acos(cos_q3)

        alpha_ang = math.atan2(dz, r_w)
        cos_beta  = max(min((D**2 + L1**2 - L2**2) / (2 * D * L1), 1.0), -1.0)
        beta = math.acos(cos_beta)
        
        q2 = alpha_ang + beta if elbow == 'up' else alpha_ang - beta
        q4 = phi_local - q2 - q3

        if not (-135.0 <= math.degrees(q1) * -1 <= 135.0): results[name] = None; continue
        if not (-60.0  <= math.degrees(q2) * -1 + 90.0 <= 240.0): results[name] = None; continue

        results[name] = (q1, q2, q3, q4)
    return results


def plot_four_configs(target_pos=(0.22, 0.0, 0.08), target_phi=0.0, lang='cn'):
    configs = solve_four_configs(target_pos, target_phi, lang)
    valid = {k: v for k, v in configs.items() if v is not None}

    n = len(valid)
    fig = plt.figure(figsize=(5 * n, 5), dpi=120)
    
    if lang == 'cn':
        title = f'目标点 {target_pos}  φ={math.degrees(target_phi):.0f}° — {n}种有效构型'
        label_target = '目标点'
    else:
        title = f'Target Pos {target_pos}  φ={math.degrees(target_phi):.0f}° — {n} Valid Configs'
        label_target = 'Target'

    fig.suptitle(title, fontsize=14, fontweight='bold')

    for idx, (name, q) in enumerate(valid.items()):
        ax = fig.add_subplot(1, n, idx + 1, projection='3d')
        plot_arm_with_frames(*q, lang=lang, title=name, ax=ax)
        ax.scatter(*target_pos, color='#e74c3c', s=80, edgecolors='white', zorder=10, label=label_target)
        ax.legend(fontsize=8, loc='upper left')

    return fig

# ─────────────────────────────────────────────────────────────────────────────
# 主程序
# ─────────────────────────────────────────────────────────────────────────────
def default_output_dir() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / 'figures' / 'kinematics'


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='生成机械臂运动学与工作空间论文配图。')
    parser.add_argument(
        '--output-dir',
        default=str(default_output_dir()),
        help='输出图片目录，默认写入仓库 figures/kinematics/',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=200000,
        help='工作空间蒙特卡洛采样点数量，默认 200000。',
    )
    return parser


def main(args=None):
    parsed = build_arg_parser().parse_args(args=args)
    output_dir = Path(parsed.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    print("🚀 正在极速采样工作空间 (向量化计算)...")
    xs, ys, zs = compute_workspace_vectorized(n_samples=parsed.samples)
    print(f"✅ 有效可达点数：{len(xs)}\n")

    for lang in ['cn', 'en']:
        print(f"🎨 正在生成【{'中文' if lang=='cn' else '英文'}版】图表...")
        
        # 图 1
        q_demo = (0.0, math.radians(30), math.radians(-60), math.radians(30))
        fig1 = plt.figure(figsize=(8, 7), dpi=150)
        plot_arm_with_frames(*q_demo, lang=lang, ax=fig1.add_subplot(111, projection='3d'))
        plt.savefig(output_dir / f'fig1_arm_frames_{lang}.png', dpi=300)
        plt.close(fig1)

        # 图 2
        fig2 = plt.figure(figsize=(9, 7), dpi=150)
        plot_workspace(xs, ys, zs, lang=lang, ax=fig2.add_subplot(111, projection='3d'))
        plt.savefig(output_dir / f'fig2_workspace_3d_{lang}.png', dpi=300)
        plt.close(fig2)

        # 图 3
        fig3, _ = plot_workspace_slice(xs, ys, zs, z_target=0.08, tolerance=0.005, lang=lang)
        plt.savefig(output_dir / f'fig3_workspace_slice_{lang}.png', dpi=300)
        plt.close(fig3)

        # 图 4
        fig4 = plot_four_configs(target_pos=(0.22, 0.05, 0.12), target_phi=0.0, lang=lang)
        plt.savefig(output_dir / f'fig4_four_configs_{lang}.png', dpi=300)
        plt.close(fig4)
        
        print(f"📸 {lang.upper()} 版本生成完毕！\n")

    print(f"🎉 所有双语渲染流程完成！文件已保存在：{output_dir}")


if __name__ == '__main__':
    main()
