import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import smbus2

class SensorInterfaceNode(Node):
    def __init__(self):
        super().__init__('sensor_interface_node')
        
        # ADS1115 配置
        self.I2C_BUS = 1
        self.ADS_ADDR = 0x48
        
        try:
            self.bus = smbus2.SMBus(self.I2C_BUS)
            self.bus.read_byte(self.ADS_ADDR)
            self.get_logger().info("✅ ADS1115 I2C 模块连接成功")
            self.is_ready = True
        except Exception as e:
            self.get_logger().error(f"❌ ADS1115 连接失败: {e}")
            self.is_ready = False

        # 发布两个薄膜传感器的受力数据 (单位: 克)
        self.fsr_left_pub = self.create_publisher(Float32, '/sensor/fsr_left', 10)
        self.fsr_right_pub = self.create_publisher(Float32, '/sensor/fsr_right', 10)

        # 20Hz 刷新率
        self.timer = self.create_timer(0.05, self.read_and_publish)

    def read_voltage(self, channel):
        if not self.is_ready: return 0.0
        mux = {0: 0xC3, 1: 0xD3}[channel]
        self.bus.write_i2c_block_data(self.ADS_ADDR, 0x01, [mux, 0x8B])
        time.sleep(0.01)
        result = self.bus.read_i2c_block_data(self.ADS_ADDR, 0x00, 2)
        raw = (result[0] << 8) | result[1]
        if raw & 0x8000: raw -= 0x10000
        return raw * (4.096 / 32768.0)

    def calc_force(self, voltage):
        if voltage > 3.1: return 0.0 # 死区
        voltage = max(0.001, min(voltage, 3.299))
        r_fsr = 10.0 * (voltage / (3.3 - voltage))
        r_fsr = max(r_fsr, 0.01)
        return (194.46 / r_fsr) ** (1.0 / 0.665)

    def read_and_publish(self):
        # 假设 AIN0 接左夹爪传感器，AIN1 接右夹爪传感器
        v_left = self.read_voltage(0)
        v_right = self.read_voltage(1)
        
        f_left = self.calc_force(v_left)
        f_right = self.calc_force(v_right)
        
        self.fsr_left_pub.publish(Float32(data=float(f_left)))
        self.fsr_right_pub.publish(Float32(data=float(f_right)))

def main(args=None):
    rclpy.init(args=args)
    node = SensorInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_ready: node.bus.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()