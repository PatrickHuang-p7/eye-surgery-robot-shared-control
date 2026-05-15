import serial
import time
import binascii

# 🚨 配置你的串口 (根据你之前的 ls -l 结果，应该是 /dev/serial0)
# 如果是用 USB 转接板，则是 /dev/ttyUSB0
SERIAL_PORT = '/dev/serial0' 
BAUDRATE = 115200

def calculate_checksum(payload):
    """计算 Fashion Star 协议的校验和"""
    # 校验和 = (Header(2byte) + Code(1byte) + Len(1byte) + Content) 的和 % 256
    # PING 指令包头固定为 0x12 0x4C
    total = 0x12 + 0x4C
    for b in payload:
        total += b
    return total % 256

def debug_tx_rx():
    print(f"--- 开始串口 TX/RX 测试 ---")
    print(f"端口: {SERIAL_PORT}, 波特率: {BAUDRATE}")
    
    try:
        # 1. 打开串口
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1) # 100ms 超时
        print(f"✅ 串口打开成功")
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        return

    try:
        # 2. 构造 PING 指令 (针对 ID 0)
        # 协议格式: [Header 2B] [Code 1B] [Len 1B] [Param (ID) 1B] [Checksum 1B]
        # Code 1 = PING
        code = 0x01
        length = 0x01
        servo_id = 3 # 假设测试 ID 0
        
        # 构造除了 Header 和 Checksum 之外的数据
        payload = bytes([code, length, servo_id])
        
        # 计算校验和
        checksum = calculate_checksum(payload)
        
        # 完整数据包: Header(0x12 0x4C) + Payload + Checksum
        packet = b'\x12\x4c' + payload + bytes([checksum])
        
        # 3. 执行 TX (发送)
        print(f"\n[TX 发送] 正在向 ID {servo_id} 发送 Ping 指令...")
        print(f"   数据: {binascii.hexlify(packet).upper()}")
        ser.write(packet)
        
        # 4. 执行 RX (接收)
        print(f"[RX 接收] 等待回复...")
        time.sleep(0.05) # 给舵机一点反应时间
        
        # 读取所有缓冲区数据
        response = ser.read(100) 
        
        if len(response) > 0:
            print(f"✅ [RX 成功] 收到 {len(response)} 字节数据:")
            print(f"   数据: {binascii.hexlify(response).upper()}")
            
            # 简单验证包头
            if response.startswith(b'\x05\x1c'):
                print("   -> 包头校验通过 (0x05 0x1C)，是有效的舵机响应！")
            elif response == packet:
                print("   -> ⚠️ 警告：收到的数据和发送的一模一样，可能是 TX/RX 短路（硬件回环）！")
            else:
                print("   -> 数据包头未知，可能是噪声或部分数据。")
        else:
            print("❌ [RX 失败] 没有收到任何数据 (超时)。")
            print("   诊断建议：")
            print("   1. 检查 RX 线：树莓派 RX (Pin 10) <--> UC-01 TX")
            print("   2. 检查共地：树莓派 GND <--> UC-01 GND (必须接！)")
            print("   3. 确认舵机 ID 是否真的是 0。")

    except Exception as e:
        print(f"运行出错: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("\n串口已关闭")

if __name__ == '__main__':
    debug_tx_rx()