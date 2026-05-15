import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import math
import serial

from .fashion_star_sdk import UartServoManager 

class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')
        self.declare_parameter('serial_port', '/dev/serial0')
        self.serial_port = self.get_parameter('serial_port').value
        
        self.servo_ids = [1, 2, 3, 4, 5] 
        self.offsets_deg = [0.0, 90.0, 45.0, 0.0, 0.0]
        self.dir_mult = [-1, -1, 1, 1, 1] 
        
        # 🌟 新增：用来缓存大脑发来的“目标物理角度”，方便对比
        self.target_phys_deg = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info(f'正在连接串口: {self.serial_port}')
        
        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=0.1)
            self.servo_mgr = UartServoManager(self.ser, is_debug=False)
            
            for sid in self.servo_ids:
                if self.servo_mgr.ping(sid):
                    self.get_logger().info(f'✅ 舵机 ID {sid} 连接成功!')
                else:
                    self.get_logger().warn(f'❌ 未检测到舵机 ID {sid}')
                    
        except serial.SerialException as e:
            self.get_logger().error(f"串口连接失败: {e}")
            self.ser = None

        self.command_subscription = self.create_subscription(
            Float32MultiArray,
            '/arm/joint_command',
            self.command_callback,
            10
        )

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        # 保持 10Hz 查询频率
        self.timer = self.create_timer(0.1, self.publish_joint_states) 
        self.joint_names = ['base_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'gripper']

    def command_callback(self, msg: Float32MultiArray):
        """处理接收到的关节角度指令 (带绝对防穿模护盾)"""
        if self.ser is None or len(msg.data) != 5:
            return

        rad_angles = msg.data 
        q1, q2, q3, q4 = rad_angles[:4]

        # ========================================================
        # 🛑 终极硬件护盾：实时拦截“砸桌子”指令
        # ========================================================
        # 植入机械臂的物理尺寸 (米)
        d1, L1, L2, L_end = 0.060, 0.0814, 0.0814, 0.140
        
        # 通过正运动学，推算指令将导致的物理高度
        z_elbow = d1 + L1 * math.sin(q2)
        z_wrist = z_elbow + L2 * math.sin(q2 + q3)
        z_tcp = z_wrist + L_end * math.sin(q2 + q3 + q4)
        
        # 设定绝对安全底线 (Z = 0.00 米即为桌面)
        TABLE_Z = 0.00
        
        if z_elbow < TABLE_Z or z_wrist < TABLE_Z or z_tcp < TABLE_Z:
            # 使用 throttle 限制报错频率，防止刷爆终端
            self.get_logger().error(
                f"🚨 硬件防撞锁触发！指令已拦截！\n"
                f"危险预测: 肘 Z={z_elbow:.3f}m, 腕 Z={z_wrist:.3f}m, 夹爪 Z={z_tcp:.3f}m",
                throttle_duration_sec=1.0
            )
            # ⚠️ 极其关键：直接 return，绝不把指令发给底层舵机！
            return 
        # ========================================================

        # 如果通过了高度安全审查，再执行后续的串口发送
        for i in range(5):
            sid = self.servo_ids[i]
            target_deg = (math.degrees(rad_angles[i]) * self.dir_mult[i]) + self.offsets_deg[i]
            
            self.target_phys_deg[i] = target_deg

            self.servo_mgr.set_servo_angle(
                servo_id=sid, 
                angle=target_deg, 
                velocity=50, 
                t_acc=20,
                t_dec=20
            )

    def publish_joint_states(self):
        if self.ser is None:
            return
            
        current_rads = []
        actual_phys_deg = [] # 🌟 暂存真实物理角度
        
        for i in range(5):
            sid = self.servo_ids[i]
            deg_angle = self.servo_mgr.query_servo_angle(sid)
            
            if deg_angle is not None:
                actual_phys_deg.append(deg_angle)
                rad_angle = math.radians((deg_angle - self.offsets_deg[i]) * self.dir_mult[i])
            else:
                actual_phys_deg.append(0.0) # 读取失败给个默认值 0
                rad_angle = 0.0 
                
            current_rads.append(rad_angle)

        # ----------------------------------------------------
        # 🌟 核心：生成终端对比看板 (使用 throttle 控制每秒打印 1 次)
        # ----------------------------------------------------
        log_str = "\n--- 🎯 物理角度硬件闭环追踪面板 (Target vs Actual) ---\n"
        for i in range(4): # 夹爪暂不重点监控，只看前 4 个驱动关节
            err = self.target_phys_deg[i] - actual_phys_deg[i]
            # 格式化输出：自动对齐，一目了然
            log_str += f" [ID {self.servo_ids[i]}] 目标: {self.target_phys_deg[i]:>6.1f}° | 真实: {actual_phys_deg[i]:>6.1f}° | 滞后误差: {err:>5.1f}°\n"
            
        # 重点预警：如果某个关节滞后误差超过 5 度，说明它可能遇到了阻力或者供电不足
        if abs(self.target_phys_deg[1] - actual_phys_deg[1]) > 5.0:
             log_str += " 🚨 警告: ID 2 (肩部) 硬件滞后严重，可能承受重力过大或供电不足！\n"

        self.get_logger().info(log_str, throttle_duration_sec=1.0)
        # ----------------------------------------------------

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = current_rads
        self.joint_state_publisher.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()