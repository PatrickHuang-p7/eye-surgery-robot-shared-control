import math
import numpy as np

class KinematicsSolver:
    def __init__(self):
        self.d1 = 0.060      
        self.L1 = 0.0814       
        self.L2 = 0.0814       
        self.L_end = 0.140   
        
    def forward_kinematics(self, q):
        q1, q2, q3, q4 = q[0], q[1], q[2], q[3]
        phi = q2 + q3 + q4
        r2 = self.L1 * math.cos(q2) + self.L2 * math.cos(q2 + q3) + self.L_end * math.cos(phi)
        x = r2 * math.cos(q1)
        y = r2 * math.sin(q1)
        z = self.d1 + self.L1 * math.sin(q2) + self.L2 * math.sin(q2 + q3) + self.L_end * math.sin(phi)
        return np.array([x, y, z]), phi

    def inverse_kinematics(self, target_pos, target_phi, config='auto'):
        """
        全自动多解 IK 引擎：计算全部 4 种构型，自动过滤物理越界解，返回最优解。
        """
        x, y, z = target_pos[0], target_pos[1], target_pos[2]
        valid_solutions = []
        
        # ========================================================
        # 🌟 1. 生成所有可能的底层意图组合：[底座朝向, 局部俯仰角, 肘部姿态]
        # ========================================================
        q1_front = math.atan2(y, x)
        r_front = math.sqrt(x**2 + y**2)
        
        q1_back = q1_front + math.pi if q1_front <= 0 else q1_front - math.pi
        r_back = -r_front
        phi_back = math.pi - target_phi # 背越式需要镜像末端姿态
        
        # 优先级排序：正向肘上 > 正向肘下 > 背越肘上 > 背越肘下
        configs_to_try = [
            (q1_front, r_front, target_phi, 'elbow_up', '正向-肘上'),
            (q1_front, r_front, target_phi, 'elbow_down', '正向-肘下'),
            (q1_back, r_back, phi_back, 'elbow_up', '背向-肘上'),
            (q1_back, r_back, phi_back, 'elbow_down', '背向-肘下')
        ]
        
        # ========================================================
        # 🌟 2. 遍历计算并进行出厂级“物理限位”压测
        # ========================================================
        for q1, r, phi_local, elbow_mode, name in configs_to_try:
            r_w = r - self.L_end * math.cos(phi_local)
            z_w = z - self.L_end * math.sin(phi_local)
            delta_z = z_w - self.d1
            
            D = math.sqrt(r_w**2 + delta_z**2)
            if D > (self.L1 + self.L2) or D < abs(self.L1 - self.L2):
                continue # 这个构型下手臂不够长，直接淘汰
                
            cos_q3 = (D**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            cos_q3 = max(min(cos_q3, 1.0), -1.0) 
            q3 = math.acos(cos_q3)
            
            if elbow_mode == 'elbow_up':
                q3 = -q3 # 肘部向上弯折 (对于当前连杆定义是负数)
                
            alpha = math.atan2(delta_z, r_w)
            cos_beta = (D**2 + self.L1**2 - self.L2**2) / (2 * D * self.L1)
            cos_beta = max(min(cos_beta, 1.0), -1.0)
            beta = math.acos(cos_beta)
            
            if elbow_mode == 'elbow_up':
                q2 = alpha + beta
            else:
                q2 = alpha - beta
                
            q4 = phi_local - q2 - q3
            
            # --- 🛑 终极硬件限位审查 ---
            id1_phys = math.degrees(q1) * -1 + 0.0
            id2_phys = math.degrees(q2) * -1 + 90.0
            
            # 你的物理底线在这里！(ID1: ±135, ID2: -60 ~ 240)
            if not (-135.0 <= id1_phys <= 135.0):
                continue
            if not (-60.0 <= id2_phys <= 240.0):
                continue
                
            # 如果没被 continue 拦截，说明这个解物理上完美可行！
            valid_solutions.append({
                'q': np.array([q1, q2, q3, q4]),
                'name': name
            })

        # ========================================================
        # 🌟 3. 智能决策：返回优先级最高的可行解
        # ========================================================
        if not valid_solutions:
            print("🛑 绝对死角：遍历了 4 种姿态，全被物理极限(ID1或ID2)拦截，此坐标绝对不可达！")
            return None
            
        best_solution = valid_solutions[0]
        # 如果不是默认的第一种(正向肘上)，打印一下它用了哪种姿态救场
        if best_solution['name'] != '正向-肘上':
            print(f"🔄 智能回退触发：原姿态受限，系统已自动切换为 [{best_solution['name']}] 姿态前往目标！")
            
        return best_solution['q']