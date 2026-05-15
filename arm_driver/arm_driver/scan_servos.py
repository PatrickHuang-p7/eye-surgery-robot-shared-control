import serial
from fashion_star_sdk import UartServoManager # 假设你在 arm_driver 文件夹下运行
import time

# 🚨 请将串口号改为你的机械臂实际连接的串口号
SERIAL_PORT = '/dev/serial0' 
BAUDRATE = 115200

def scan_servos():
    uart = None
    try:
        # 初始化串口连接
        uart = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=0.01)
        uservo = UartServoManager(uart, is_debug=False)
        
        print(f"--- Scanning for servos on {SERIAL_PORT} ---")
        
        # 调用 SDK 的扫描功能
        uservo.scan_servo() 
        
        servo_list = list(uservo.servos.keys())
        
        if servo_list:
            print(f"\n✅ Found {len(servo_list)} servos:")
            print(f"IDs: {servo_list}")
            
            # --- 额外：通过设置角度来确认 ID ---
            print("\n💡 正在测试每个舵机，请观察机械臂动作：")
            for servo_id in servo_list:
                print(f"  - 正在移动舵机 ID {servo_id}...")
                
                # 设置 ID 舵机角度到 30 度
                uservo.set_servo_angle(servo_id, 60.0, interval=500)
                time.sleep(1) 
                
                # 设置 ID 舵机角度到 0 度
                uservo.set_servo_angle(servo_id, 0.0, interval=500)
                time.sleep(1) 
                
        else:
            print("❌ No servos found. Check wiring and power supply.")
            
    except serial.SerialException as e:
        print(f"❌ Failed to open serial port {SERIAL_PORT}: {e}")
    finally:
        if uart and uart.is_open:
            uart.close()

if __name__ == '__main__':
    scan_servos()