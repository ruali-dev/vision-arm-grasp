import serial
import json
import time
import ctypes
import os

# 加载运动学计算库
# 编译运动学计算库gcc -shared -o calculate_servo_angles.so -fPIC calculate_servo_angles.cpp -lm
if os.name == "nt":  # Windows
    lib = ctypes.CDLL("./calculate_servo_angles.dll")
else:  # Linux/macOS
    lib = ctypes.CDLL("./calculate_servo_angles.so")

# 定义函数返回类型（返回一个结构体）
class ServoAngles(ctypes.Structure):
    _fields_ = [
        ("servo1", ctypes.c_float),
        ("servo2", ctypes.c_float),
        ("servo3", ctypes.c_float),
        ("servo4", ctypes.c_float),
    ]

# 设置函数参数和返回类型
lib.calculate_servo_angles.argtypes = [
    ctypes.c_float,  # X
    ctypes.c_float,  # Y
    ctypes.c_float,  # Z
]
lib.calculate_servo_angles.restype = ServoAngles

def send_json_to_arduino(port, baudrate, angles):
    try:
        ser = serial.Serial(port, baudrate, timeout=5, bytesize=8, parity='N', stopbits=1)  # 设置串口参数
        time.sleep(3)
        print(f"已打开串口: {port}")
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
        return

    # 生成 JSON 数据
    control_command = {
        "servo1": round(angles.servo1, 2),
        "servo2": round(angles.servo2, 2),
        "servo3": round(angles.servo3, 2),
        "servo4": 145,
        "servo5": round(angles.servo4, 2),  # 这个才是 j4
        "servo6": 90  # 夹爪先不用
    }
    json_data = json.dumps(control_command, separators=(',', ':')) + '\n'  # 紧凑格式 + 换行符

    # 清空串口缓冲区
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # 发送 JSON 数据
    try:
        ser.write(json_data.encode('utf-8'))  # 发送数据
        time.sleep(3)
        print(f"已发送 JSON 数据: {json_data.strip()}")

        response = ser.readline().decode('utf-8').strip()
        print(f"Arduino 返回的数据: {response}")

    except Exception as e:
        print(f"发送数据失败: {e}")

    # 关闭串口
    ser.close()
    print("串口已关闭")
    return  # 确保函数退出

def reset_arm(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=5, bytesize=8, parity='N', stopbits=1)  # 设置串口参数
        time.sleep(3)
        print(f"已打开串口: {port}")
    except serial.SerialException as e:
        print(f"无法打开串口: {e}")
        return

    # 生成 JSON 数据，所有舵机角度均为90度
    control_command = {
        "servo1": 90,
        "servo2": 90,
        "servo3": 90,
        "servo4": 145,
        "servo5": 90,
        "servo6": 90  # 夹爪
    }
    json_data = json.dumps(control_command, separators=(',', ':')) + '\n'  # 紧凑格式 + 换行符

    # 清空串口缓冲区
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # 发送 JSON 数据
    try:
        ser.write(json_data.encode('utf-8'))  # 发送数据
        time.sleep(3)
        print(f"已发送 JSON 数据: {json_data.strip()}")
        
        response = ser.readline().decode('utf-8').strip()
        print(f"Arduino 返回的数据: {response}")

    except Exception as e:
        print(f"发送数据失败: {e}")

    # 关闭串口
    ser.close()
    print("串口已关闭")
    print("机械臂复位")

# 主函数
def main():
    # 输入目标点坐标
    x = float(input("请输入目标点 X 坐标: "))
    y = float(input("请输入目标点 Y 坐标: "))
    z = float(input("请输入目标点 Z 坐标: "))

    # 计算舵机角度
    try:
        angles = lib.calculate_servo_angles(x, y, z)
        print(f"计算出的舵机角度: j1={angles.servo1}, j2={angles.servo2}, j3={angles.servo3}, j4={angles.servo4}")
    except Exception as e:
        print(f"无法计算出舵机角度: {e}")
        return

    # 发送 JSON 数据到 Arduino
    #Ubuntu2204记得删brltty
    port = '/dev/robotarm'  # 串口号，Windows 通常是 COMx，Linux/macOS 是 /dev/ttyUSBx 或 /dev/ttyACMx
    baudrate = 115200  # 波特率，与 Arduino 代码一致
    send_json_to_arduino(port, baudrate, angles)
    #reset_arm(port,baudrate)

# 运行主函数
if __name__ == "__main__":
    main()
