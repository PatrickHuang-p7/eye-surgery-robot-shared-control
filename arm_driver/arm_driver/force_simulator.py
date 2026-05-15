import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading

class ForceSimulator(Node):
    def __init__(self):
        super().__init__('force_simulator')
        # 创建与控制节点完全对应的发布者
        self.pub = self.create_publisher(Float32MultiArray, '/force_sensor', 10)
        # 以 50Hz (0.02s) 的高频发送力数据，模拟真实的传感器数据流
        self.timer = self.create_timer(0.02, self.publish_force)
        
        self.fx = 0.0
        self.fy = 0.0

    def publish_force(self):
        msg = Float32MultiArray()
        # 严格遵守 [Fx, Fy, Fz] 的格式，Z轴固定发 0.0
        msg.data = [float(self.fx), float(self.fy), 0.0]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForceSimulator()
    
    # 使用多线程：让 ROS 2 在后台狂发数据，主线程留给用户输入
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    print("\n" + "="*45)
    print(" 🎮 虚拟力觉注入终端已启动！(纯 XY 平面)")
    print("="*45)
    print(" 👉 玩法说明:")
    print(" 1. 输入两个数字 (中间空格) 代表 Fx 和 Fy。例如: 2.0 0.0")
    print(" 2. 输入 0 0 代表松开摇杆 (由于导纳控制特性，机械臂会停在原地)")
    print(" 3. 死区已设为 0.5N，输入的力必须 > 0.5 或 < -0.5 才会移动")
    print(" 4. 输入 q 退出程序")
    print("="*45 + "\n")

    try:
        while True:
            cmd = input("🟢 请输入力向量 [Fx Fy]: ")
            if cmd.lower() == 'q':
                break
            
            parts = cmd.split()
            if len(parts) >= 2:
                try:
                    node.fx = float(parts[0])
                    node.fy = float(parts[1])
                    print(f"   ⚡ 已持续注入恒定推力 -> Fx: {node.fx}N, Fy: {node.fy}N")
                except ValueError:
                    print("   ❌ 输入格式错误，请输入数字！")
            else:
                print("   ❌ 请输入两个数字，用空格隔开！")
                
    except KeyboardInterrupt:
        pass
    finally:
        print("\n🛑 正在关闭力觉注入器...")
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()