import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time


class ActionDemo(Node):
    def __init__(self):
        super().__init__('action_demo_node')
        # 创建发布者，发送给驱动节点
        self.pub = self.create_publisher(Float32MultiArray, '/arm/joint_command', 10)
        self.get_logger().info("动作序列节点已启动，准备开始表演...")

    def send_joints(self, angles, duration=3.0):
        """
        发送关节角度并等待动作完成
        :param angles: [Base, Shoulder, Elbow, Wrist, Gripper] (单位: 弧度)
        :param duration: 等待时间 (秒)
        """
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.pub.publish(msg)
        self.get_logger().info(f"执行动作: {angles}")
        time.sleep(duration)

    def run_sequence(self):
        # 机械臂动作编排，单位为弧度。
        # 顺序: [基座, 大臂, 小臂, 手腕, 夹爪]
        
        # 1. 【预备】全关节归零 (立正)
        # 此时机械臂应该笔直向上或折叠
        self.get_logger().info(">>> 1. 归零复位")
        self.send_joints([0.0, 0.0, 0.0, 0.0, 0.0], duration=2.0)

        # 2. 【热身】基座旋转测试
        self.get_logger().info(">>> 2. 摇头 (基座旋转)")
        self.send_joints([0.5, 0.0, 0.0, 0.0, 0.0])  # 左转约 30度
        self.send_joints([-0.5, 0.0, 0.0, 0.0, 0.0]) # 右转约 30度
        self.send_joints([0.0, 0.0, 0.0, 0.0, 0.0])  # 回中

        # 3. 【伸展】前伸动作 (去抓东西的姿势)
        self.get_logger().info(">>> 3. 伸手 (前往抓取点)")
        # 假设：大臂前倾(0.5), 小臂放平(1.0), 手腕调整
        self.send_joints([0.0, 0.5, 1.0, -1.5, 0.0], duration=2.0)

        # 4. 【抓取】夹爪测试
        self.get_logger().info(">>> 4. 抓取测试")
        self.send_joints([0.0, 0.5, 1.0, -1.5, 1.0]) # 张开 (假设 1.0 是开)
        time.sleep(1.0)
        self.send_joints([0.0, 0.5, 1.0, -1.5, 0.0]) # 闭合 (假设 0.0 是关)
        time.sleep(1.0)
        
        # 5. 【收回】模拟抓到东西后举起来
        self.get_logger().info(">>> 5. 举起物体")
        self.send_joints([0.0, 0.0, 0.5, -0.5, 0.0], duration=2.0)

        # 6. 【结束】回到安全位置
        self.get_logger().info(">>> 6. 演示结束，回零")
        self.send_joints([0.0, 0.0, 0.0, 0.0, 0.0])

def main(args=None):
    rclpy.init(args=args)
    node = ActionDemo()
    
    try:
        # 给一点时间让 ROS 网络建立连接
        time.sleep(1.0)
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
