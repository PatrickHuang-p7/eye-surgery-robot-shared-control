import time
import smbus2

# ==========================================
# ADS1115 配置参数
# ==========================================
I2C_BUS = 1
ADS1115_ADDRESS = 0x48
REG_CONVERSION = 0x00
REG_CONFIG = 0x01
ADS_FSR = 4.096 
ADS_RESOLUTION = 32768.0

# 初始化 I2C 总线
try:
    bus = smbus2.SMBus(I2C_BUS)
    bus.read_byte(ADS1115_ADDRESS) # 探测一下地址
    print("✅ 成功连接到 ADS1115 (I2C 地址 0x48)")
except Exception as e:
    print(f"❌ 连接 ADS1115 失败: {e}")
    print("请检查线缆，并确保终端运行过 'i2cdetect -y 1' 能看到 48")
    exit(1)

def read_adc_voltage(channel):
    """读取指定通道 (0-3) 的电压值"""
    mux = {
        0: 0xC3, # AIN0 压力传感器 1
        1: 0xD3, # AIN1 压力传感器 2
        2: 0xE3, # AIN2 摇杆 X
        3: 0xF3  # AIN3 摇杆 Y
    }[channel]

    # 写入配置：单次转换，128SPS，PGA 4.096V
    bus.write_i2c_block_data(ADS1115_ADDRESS, REG_CONFIG, [mux, 0x8B])
    time.sleep(0.01) # 等待转换完成

    # 读取 2 字节数据
    result = bus.read_i2c_block_data(ADS1115_ADDRESS, REG_CONVERSION, 2)
    raw_adc = (result[0] << 8) | result[1]
    
    # 转换为有符号数
    if raw_adc & 0x8000:
        raw_adc -= 0x10000
        
    # 计算实际电压
    return raw_adc * (ADS_FSR / ADS_RESOLUTION)

def calculate_force(voltage):
    """
    基于传感器原厂公式与分压电路的真实受力解算
    """
    V_IN = 3.3 # 树莓派传感器的参考供电电压
    R_PULLUP = 10.0 # 模块上的上拉电阻，通常为 10 kOhm

    # 1. 设置死区电压，消除底噪 (没碰时电压大约在 3.1V ~ 3.3V 波动)
    if voltage > 3.1:
        return 0.0
        
    # 防止因底噪导致电压大于 V_IN 产生的除负数错误，以及电压为 0 导致的除零错误
    voltage = max(0.001, min(voltage, V_IN - 0.001))

    # 2. 根据分压原理，计算薄膜传感器当前的真实电阻 (单位: kOhm)
    r_fsr = R_PULLUP * (voltage / (V_IN - voltage))

    # 设置一个最小电阻下限，防止用力过猛算出无限大的力
    r_fsr = max(r_fsr, 0.1)

    # 3. 代入原厂逆向公式，计算真实受力 (单位: 克)
    # X = (194.46 / Y) ^ (1 / 0.665)
    force_grams = (194.46 / r_fsr) ** (1 / 0.665)
    
    return force_grams

print("\n🚀 开始读取传感器数据... (按 Ctrl+C 退出)\n")
print(f"{'通道':<8} | {'原始电压 (V)':<12} | {'换算压力 (Y)':<15}")
print("-" * 45)

try:
    while True:
        # 我们先只测 A0 通道（假设你把其中一个薄膜传感器接在 A0 上）
        v_a0 = read_adc_voltage(0)
        
        # 套用公式
        force_y = calculate_force(v_a0)
        
        # 实时打印（使用回车符 \r 刷新同一行）
        print(f"AIN0     | {v_a0:>8.4f} V   | {force_y:>10.2f}    ", end='\r')
        
        time.sleep(0.1) # 10Hz 刷新率

except KeyboardInterrupt:
    print("\n\n🛑 测试已手动停止。")
finally:
    bus.close()