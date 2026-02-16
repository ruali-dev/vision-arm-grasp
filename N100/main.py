import serial
import json
import time
import ctypes
import os
import numpy as np
import cv2
import threading

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


class RobotArm:
    #机械臂类
    def __init__(self,port,baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    #激活机械臂链接
    def active(self):
        try:
            self.ser = serial.Serial(self.port,self.baudrate, timeout=5, bytesize=8, parity='N', stopbits=1)
            time.sleep(3)
            print(f"已打开串口: {self.port},机械臂链接已激活")
        except serial.SerialException as e:
            print(f"无法打开串口: {e}")
            return

    #关闭机械臂链接    
    def shutdown(self):
        # 检查 ser 是否已经被初始化
        if self.ser is not None:
            # 清空串口缓冲区
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.close()
            self.ser = None  # 将 ser 重置为 None
            print("串口已关闭, 机械臂断开链接")
        else:
            print("串口尚未打开或已经关闭") 

    #发送控制json
    def send_command(self, angles, servo6_value):
        command = {
            "servo1": round(angles.servo1, 2),
            "servo2": round(angles.servo2, 2),
            "servo3": round(angles.servo3, 2),
            "servo4": 145,
            "servo5": round(angles.servo4, 2),  # 确认这是 j4
            "servo6": servo6_value
        }
        json_data = json.dumps(command, separators=(',', ':')) + '\n'  # 紧凑格式 + 换行符
        
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        try:
            self.ser.write(json_data.encode('utf-8'))  # 发送数据
            time.sleep(3)  # 根据实际通信情况调整时间
            print(f"已发送 JSON 数据: {json_data.strip()}")
            
            response = self.ser.readline().decode('utf-8').strip()
            print(f"Arduino 返回的数据: {response}")
            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False

    def catch(self, x, y):
        angles = lib.calculate_servo_angles(x, y, 5)
        if self.send_command(angles, 180):
            time.sleep(2)
            self.send_command(angles,47)
            time.sleep(2)
            return True
        return False

    def throw(self):
        angles = lib.calculate_servo_angles(6, 10, 18)
        self.send_command(angles, 47)
        time.sleep(3)
        angles = lib.calculate_servo_angles(31, -4 , 18)
        self.send_command(angles, 47)
        time.sleep(3)
        self.send_command(angles, 180)
        return True
    
    def reset(self):
        command = {
            "servo1": 90,
            "servo2": 90,
            "servo3": 90,
            "servo4": 145,
            "servo5": 90,  # 确认这是 j4
            "servo6": 180
        }
        json_data = json.dumps(command, separators=(',', ':')) + '\n'  # 紧凑格式 + 换行符
        
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        try:
            self.ser.write(json_data.encode('utf-8'))  # 发送数据
            time.sleep(3)  # 根据实际通信情况调整时间
            print(f"已发送 JSON 数据: {json_data.strip()}")
            
            response = self.ser.readline().decode('utf-8').strip()
            print(f"Arduino 返回的数据: {response}")
            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False

class Camera:
    def __init__(self, device=0):
        self.cap = cv2.VideoCapture(device)
        self.colors = {
            'yellow': {'lower': np.array([29, 0, 0]), 'upper': np.array([180, 255, 255])},
            'pink': {'lower': np.array([122, 82, 102]), 'upper': np.array([180, 255, 255])},
            'white': {'lower': np.array([109, 0, 223]), 'upper': np.array([180, 255, 255])}
        }
        self.object_centers = {}
        self.lock = threading.Lock()
        self.running = False
        self.processing_enabled = True
        self.processing_lock = threading.Lock()
        self.object_detected = threading.Event()  # 添加事件

    def start(self):
        if not self.running:
            self.running = True
            # 添加延迟，确保摄像头稳定
            time.sleep(1)
            thread = threading.Thread(target=self.active)
            thread.start()

    def active(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("摄像头无法成功读取")
                continue
            flipped_frame = cv2.flip(frame, 0)
            hsv = cv2.cvtColor(flipped_frame, cv2.COLOR_BGR2HSV)
            
            with self.processing_lock:
                processing = self.processing_enabled
            
            if processing:
                self.object_centers = {}
                result_frame = self.scan_objects(hsv, flipped_frame)
            else:
                result_frame = flipped_frame  # 不进行物体检测，直接显示帧
            
            cv2.imshow('Camera feedback', result_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
        self.cap.release()
        cv2.destroyAllWindows()

    def stop(self):
        self.running = False
        # 等待摄像头线程结束
        if self.thread and self.thread.is_alive():
            self.thread.join()

    def scan_objects(self, hsv, frame):
        for color, thresholds in self.colors.items():
            mask_color = cv2.inRange(hsv, thresholds['lower'], thresholds['upper'])
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_color, connectivity=8)
            for i in range(1, num_labels):
                x, y, w, h, area = stats[i]
                center_x, center_y = centroids[i]
                if area < 150:
                    continue
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)
                with self.lock:
                    if color not in self.object_centers:
                        self.object_centers[color] = []
                    self.object_centers[color].append((int(center_x), int(center_y)))
                    if color == 'white':
                        self.object_detected.set()  # 检测到白色物体时设置事件
        return frame

    def get_object_centers(self):
        with self.lock:
            return self.object_centers.copy()

    def calculate_coordinates(self, image_x, image_y):
        scale_factor = 0.0460
        image_center = np.array([320, 240])
        theta = np.radians(-90)
        T = np.array([-0.5, 30.1])
        u, v = image_x, image_y
        P_pixel = np.array([u, v])
        P_prime = (P_pixel - image_center) * scale_factor
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        P = T + R @ P_prime
        print(f"像素坐标 (u, v) = ({u}, {v}) 转换为机械臂坐标系 (X, Y) = ({P[0]:.2f}, {P[1]-15:.2f}) cm")
        return P[0], P[1]-15


def main():
    myarm = RobotArm(port, baudrate)
    myarm.active()
    cam = Camera()
    cam.start()
    
    # 等待物体被检测到
    cam.object_detected.wait()
    
    # 获取物体坐标
    centers = cam.get_object_centers()
    if 'white' in centers:
        for x, y in centers['white']:
            arm_x, arm_y = cam.calculate_coordinates(x, y)
            with cam.processing_lock:
                    cam.processing_enabled = False
            catched = myarm.catch(arm_x,arm_y)
            if catched :
                myarm.throw()
                myarm.reset()

    if 'pink' in centers:
        for x, y in centers['pink']:
            arm_x, arm_y = cam.calculate_coordinates(x, y)
            with cam.processing_lock:
                    cam.processing_enabled = False
            catched = myarm.catch(arm_x,arm_y)
            if catched :
                myarm.throw()
                myarm.reset()

    if 'yellow' in centers:
        for x, y in centers['yellow']:
            arm_x, arm_y = cam.calculate_coordinates(x, y)
            with cam.processing_lock:
                    cam.processing_enabled = False
            catched = myarm.catch(arm_x,arm_y)
            if catched :
                myarm.throw()
                myarm.reset()                  
    # 如果没有检测到物体，或者抓取失败，退出
    myarm.shutdown()
    cam.stop()
     
#Ubuntu2204记得删brltty
port = '/dev/robotarm'  # 串口号，Windows 通常是 COMx，Linux/macOS 是 /dev/ttyUSBx 或 /dev/ttyACMx
baudrate = 115200  # 波特率，与 Arduino 代码一致

# 运行主函数
if __name__ == "__main__":
    main()