import time
import serial
from fashion_star_sdk import UartServoManager

def calibrate():
    serial_port = '/dev/serial0' # 请确认这是你树莓派连接舵机板的串口
    servo_ids = range(7)  # 你的 5 个舵机 ID
    
    print(f"正在打开串口 {serial_port} ...")
    try:
        ser = serial.Serial(serial_port, 115200, timeout=0.1)
        servo_mgr = UartServoManager(ser, is_debug=False)
    except Exception as e:
        print(f"串口打开失败，请检查连线和权限: {e}")
        return

    print("\n--- 物理零位标定程序 ---")
    print("请确保机械臂处于掉电状态（或未发力状态）。")
    print("请用手将机械臂【水平笔直向前方伸展】。")
    input("摆好姿势后，按【回车键】开始读取当前角度...")

    offsets = []
    for sid in servo_ids:
        # 唤醒舵机
        if servo_mgr.ping(sid):
            # 禁用力矩，防止它乱动
            servo_mgr.disable_torque(sid)
            time.sleep(0.05)
            # 读取当前角度
            angle = servo_mgr.query_servo_angle(sid)
            if angle is not None:
                offsets.append(round(angle, 2))
                print(f"舵机 ID {sid} 当前物理角度: {angle:.2f}°")
            else:
                print(f"舵机 ID {sid} 读取失败！填入 0.0")
                offsets.append(0.0)
        else:
            print(f"舵机 ID {sid} 未连接！填入 0.0")
            offsets.append(0.0)

    print("\n=====================================")
    print("标定完成！请将以下数组复制并替换 arm_driver_node.py 中的 self.offsets_deg：")
    print(f"self.offsets_deg = {offsets}")
    print("=====================================\n")

if __name__ == '__main__':
    calibrate()