import numpy as np
import math
from kinematics_solver import KinematicsSolver

def run_tests():
    solver = KinematicsSolver()
    print("========== 机械臂运动学基础验证 ==========\n")

    # ---------------------------------------------------------
    # 测试 1：零位正运动学 (FK) 验证
    # 理论上，当所有关节角为 0 时，机械臂应该是水平向前完全伸直的。
    # X 应该是所有连杆长度之和，Z 应该是底座高度。
    # ---------------------------------------------------------
    q_zero = np.array([0.0, 0.0, 0.0, 0.0])
    xyz_zero, phi_zero = solver.forward_kinematics(q_zero)
    
    print("【测试 1：零点位置 (伸直状态)】")
    print(f"输入关节角 (弧度): {q_zero}")
    print(f"解算末端坐标 (米): X={xyz_zero[0]:.4f}, Y={xyz_zero[1]:.4f}, Z={xyz_zero[2]:.4f}")
    print(f"解算末端俯仰角 : {math.degrees(phi_zero):.2f}°")
    
    # 理论值计算
    expected_x = solver.L1 + solver.L2 + solver.L_end
    expected_z = solver.d1
    print(f"(理论期望值: X={expected_x:.4f}, Y=0.0000, Z={expected_z:.4f})\n")

    # ---------------------------------------------------------
    # 测试 2：逆运动学 (IK) 验证
    # 给定一个空间点，算出一组角度，再把这组角度放回 FK 里，看能不能回到原点。
    # ---------------------------------------------------------
    print("【测试 2：IK/FK 闭环验证】")
    target_x = 0.20  # 前方 15cm
    target_y = 0.050  # 左侧 5cm
    target_z = 0.100  # 高度 10cm
    target_phi = 0.0  # 要求夹爪保持水平 (0度)
    
    target_pos = np.array([target_x, target_y, target_z])
    
    print(f"设定的目标坐标: X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}")
    print(f"设定的目标俯仰角: {target_phi:.2f} 弧度")
    
    # 1. 跑逆解 (IK)
    q_ik = solver.inverse_kinematics(target_pos, target_phi, config='elbow_up')
    
    if q_ik is not None:
        print(f"IK 算出的关节角 (弧度): [{q_ik[0]:.4f}, {q_ik[1]:.4f}, {q_ik[2]:.4f}, {q_ik[3]:.4f}]")
        print(f"IK 算出的关节角 (角度): [{math.degrees(q_ik[0]):.1f}°, {math.degrees(q_ik[1]):.1f}°, {math.degrees(q_ik[2]):.1f}°, {math.degrees(q_ik[3]):.1f}°]")
        
        # 2. 跑正解 (FK) 验证
        xyz_verify, phi_verify = solver.forward_kinematics(q_ik)
        print(f"将算出的角度代回 FK: X={xyz_verify[0]:.4f}, Y={xyz_verify[1]:.4f}, Z={xyz_verify[2]:.4f}")
        
        # 计算误差
        error = np.linalg.norm(target_pos - xyz_verify)
        print(f"运动学闭环误差: {error*1000:.4f} 毫米\n")
        if error < 1e-4:
            print("✅ 验证通过：逆运动学与正运动学完美闭环！")
        else:
            print("❌ 验证失败：存在数学误差！")
    else:
        print("❌ IK 无解：设定的点超出了机械臂的物理工作空间！")

if __name__ == "__main__":
    run_tests()