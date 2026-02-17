# 视觉抓取机械臂

基于视觉引导的机械臂抓取系统，使用 OpenCV 进行目标检测，通过串口通信控制 Arduino 驱动的 6 轴机械臂。

![效果展示](assets/grasp.gif)

## 硬件配置

- **上位机**：N100 工控机（Ubuntu 22.04）
- **控制器**：Arduino UNO
- **舵机**：MG996 × 3 + MG90S × 3（共 6 个舵机）
- **摄像头**：USB 摄像头
- **机械臂**：3D 打印件组装

## 项目结构

```
vision-arm-grasp/
├── Arduino/                    # Arduino 固件
│   ├── src/main.cpp           # 主程序，接收 JSON 命令控制舵机
│   ├── include/mycontrol.h    # 头文件
│   └── platformio.ini         # PlatformIO 配置
└── N100/                      # 上位机程序（Python）
    ├── main.py                # 主程序，整合视觉和运动控制
    ├── arm.py                 # 机械臂控制类
    ├── cam.py                 # 摄像头和目标检测
    ├── calculate_servo_angles.cpp  # 逆运动学解算（C++）
    ├── coordinates_cal.py     # 像素坐标到机械臂坐标转换
    ├── color_analysis.py      # 颜色阈值调试工具
    └── camera_calibration.py  # 相机标定
```

## 工作原理

整体流程：摄像头采集 → 图像处理得到目标中心 → 坐标映射 → 逆运动学解算 → 串口下发指令 → 舵机执行抓取投放。

### 1. 目标检测
使用 OpenCV 进行颜色识别。通过 HSV 颜色空间过滤和连通域分析定位目标物体中心。

### 2. 坐标转换
摄像头获取的像素坐标通过坐标变换矩阵转换为机械臂基坐标系中的位置：

```
P = T + R × (P_pixel - image_center) × scale_factor
```

### 3. 逆运动学
基于三角函数进行逆运动学解算，将目标位置 (X, Y, Z) 转换为四个关节角度。C++ 实现的运动学解算算法通过遍历求解，满足关节角度约束条件（0°~90°）。C++ 实现为动态库，通过 Python 调用，提高解算效率。


### 4. 通信协议
Python 上位机通过串口发送 JSON 格式指令：

```json
{"servo1":90,"servo2":90,"servo3":90,"servo4":145,"servo5":90,"servo6":180}
```

Arduino 解析 JSON 并控制对应舵机。

## 编译与运行

### 编译 C++ 运动学库

```bash
cd N100
gcc -shared -o calculate_servo_angles.so -fPIC calculate_servo_angles.cpp -lm
```

### 上传 Arduino 固件

使用 PlatformIO：
```bash
cd Arduino
pio run --target upload
```

### 运行主程序

```bash
cd N100
python main.py
```

## 串口配置

- 端口：`/dev/robotarm`（Linux）或 `COMx`（Windows）
- 波特率：115200
- 注意：Linux 下可能需要移除 brltty 或创建符号链接


## 购买清单

| 配件 | 数量 |
|------|------|
| MG90S 舵机 | 3 |
| MG996 舵机（180°） | 3 |
| Arduino UNO | 1 |
| 5V 3A 电源适配器 | 1 |
| 舵机延长线 | 若干 |
| M3 螺丝、螺母、垫圈 | 若干 |
