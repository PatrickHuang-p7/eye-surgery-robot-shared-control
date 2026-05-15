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
    """
    终极状态位轮询版：不看时间，只看状态，彻底杜绝没算完就读
    """
    # 1. 科学构建配置寄存器 (恢复 128SPS)
    config = 0x0000
    config |= (1 << 15)                  # OS: 1 = 开始单次转换
    config |= ((4 + channel) << 12)      # MUX: 切换通道
    config |= (1 << 9)                   # PGA: ±4.096V
    config |= (1 << 8)                   # MODE: 单次转换模式
    config |= (4 << 5)                   # DR: 128 SPS (需要约 7.8ms)
    config |= 0x0003                     # COMP_QUE: 禁用比较器

    config_h = (config >> 8) & 0xFF
    config_l = config & 0xFF

    # 2. 发送指令：切换通道并开始转换
    bus.write_i2c_block_data(ADS1115_ADDRESS, REG_CONFIG, [config_h, config_l])

    # 3. 🌟 核心杀招：死循环盯防，直到 OS 位变成 1 (转换彻底完成)
    while True:
        status = bus.read_i2c_block_data(ADS1115_ADDRESS, REG_CONFIG, 2)
        # status[0] 是高 8 位，0x80 对应二进制的 10000000，即 OS 位
        if status[0] & 0x80: 
            break  # 转换完成，跳出循环！
        time.sleep(0.001) # 歇 1 毫秒再问，防止把 CPU 占满

    # 4. 现在去读，拿到的 100% 是当前通道最新、最干净的数据！
    result = bus.read_i2c_block_data(ADS1115_ADDRESS, REG_CONVERSION, 2)
    raw_adc = (result[0] << 8) | result[1]
    
    if raw_adc & 0x8000:
        raw_adc -= 0x10000
        
    return raw_adc * (ADS_FSR / ADS_RESOLUTION)

def calculate_force(voltage):
    """基于传感器原厂公式与分压电路的真实受力解算"""
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
    # 原始公式: Y = 194.46 * X^(-0.665)
    force_grams = (194.46 / r_fsr) ** (1 / 0.665)
    
    return force_grams

print("\n🚀 开始读取双侧传感器数据... (按 Ctrl+C 退出)\n")
print(f"{'👈 左侧 (AIN0)':<22} || {'👉 右侧 (AIN1)':<22}")
print(f"{'电压 (V)':<10} | {'受力 (g)':<9} || {'电压 (V)':<10} | {'受力 (g)':<9}")
print("-" * 48)

try:
    while True:
        # 读取 A0 (左) 和 A1 (右) 通道
        v_left = read_adc_voltage(0)
        v_right = read_adc_voltage(1)
        
        # 套用公式换算
        force_left = calculate_force(v_left)
        force_right = calculate_force(v_right)
        
        # 实时打印（使用回车符 \r 刷新同一行，加点空格清理尾部残影）
        print(f"{v_left:>8.4f}   | {force_left:>7.1f}   || {v_right:>8.4f}   | {force_right:>7.1f}    ", end='\r')
        
        time.sleep(0.1) # 10Hz 刷新率

except KeyboardInterrupt:
    print("\n\n🛑 测试已手动停止。")
finally:
    bus.close()