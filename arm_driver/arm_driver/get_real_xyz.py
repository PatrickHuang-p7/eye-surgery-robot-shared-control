import time
import math
import serial
import numpy as np
from fashion_star_sdk import UartServoManager
from kinematics_solver import KinematicsSolver

def measure_xyz():
    # 你的物理参数
    serial_port = '/dev/serial0'
    servo_ids = [1, 2, 3, 4]
    offsets_deg = [-0.8, 91.7, 49.3, -1.5] # 你的标定值
    dir_mult = [-1, -1, 1, 1]             # 你的方向标定值
    
    try:
        ser = serial.Serial(serial_port, 115200, timeout=0.1)
        servo_mgr = UartServoManager(ser, is_debug=False)
    except Exception as e:
        print(f"串口错误: {e}")
        return

    kinematics = KinematicsSolver()

    print("\n正在连接并解除机械臂力矩 (保持通电)...")
    for sid in servo_ids:
        if servo_mgr.ping(sid):
            # ⚠️ 核心修正：发送软件卸力指令
            # method 0x10 代表“停止后卸力(失锁)”，power 0 代表保持功率为 0
            servo_mgr.stop_on_control_mode(sid, 0x10, 0)
            print(f"舵机 {sid} 已成功卸力，现在可以用手掰动了。")
        else:
            print(f"❌ 舵机 {sid} 连接失败！")
            return

    print("\n==========================================")
    print("🤖 机械臂现在是“软”的，但仍处于在线监测状态。")
    print("👉 请用手将夹爪移动并摆放到你期望抓取摇杆的完美姿态。")
    input("摆好姿势后，保持手扶稳，按下【回车键】读取坐标...")

    current_rads = []
    print("\n📊 各个关节的底层真实数据：")
    for i in range(4):
        sid = servo_ids[i]
        deg_angle = servo_mgr.query_servo_angle(sid)
        
        if deg_angle is not None:
            rad_angle = math.radians((deg_angle - offsets_deg[i]) * dir_mult[i])
            current_rads.append(rad_angle)
            # 🔍 核心诊断打印：看看底层到底读到了什么鬼数据！
            print(f"舵机 ID {sid} | 物理读数: {deg_angle:>6.2f}° | 换算后弧度: {rad_angle:>6.3f} rad")
        else:
            print(f"❌ 舵机 {sid} 角度读取失败！")
            return

    # 跑一遍正运动学 (FK)
    xyz, phi = kinematics.forward_kinematics(current_rads)
    
    print("\n✅ 读取成功！")
    print("==========================================")
    print(f"🎯 摇杆当前的真实物理坐标 (请将其填入 arm_control_node.py):")
    print(f"self.joystick_pos = np.array([{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}])")
    print(f"此时末端俯仰角 Phi: {phi:.3f} 弧度")
    print("==========================================\n")

if __name__ == '__main__':
    measure_xyz()