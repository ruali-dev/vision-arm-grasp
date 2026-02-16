# Vision-Based Robotic Arm Grasping

A vision-guided robotic arm grasping system using OpenCV for object detection, with an Arduino-controlled 6-DOF robotic arm via serial communication.

![Demo](assets/grasp.gif)

**Language: [English](./README.md) | [中文](./README_zh.md)**

## Hardware

- **Host PC**: N100 industrial PC (Ubuntu 22.04)
- **Controller**: Arduino UNO
- **Servos**: MG996 × 3 + MG90S × 3 (6 servos total)
- **Camera**: USB webcam
- **Arm**: 3D printed parts

## Project Structure

```
vision-arm-grasp/
├── Arduino/                    # Arduino firmware
│   ├── src/main.cpp           # Main program, receives JSON commands
│   ├── include/mycontrol.h    # Header file
│   └── platformio.ini         # PlatformIO configuration
└── N100/                      # Host PC programs (Python)
    ├── main.py                # Main program, integrates vision and control
    ├── arm.py                 # Robot arm control class
    ├── cam.py                 # Camera and object detection
    ├── calculate_servo_angles.cpp  # Inverse kinematics (C++)
    ├── coordinates_cal.py     # Pixel to arm coordinate conversion
    ├── color_analysis.py      # Color threshold tuning tool
    └── camera_calibration.py  # Camera calibration
```

## How It Works

Flow: Camera capture → Image processing to get object center → Coordinate mapping → Inverse kinematics → Serial command → Servo execution (grab → place → reset).

### 1. Object Detection
OpenCV-based color recognition. HSV color space filtering and connected component analysis locate the object center.

### 2. Coordinate Transformation
Pixel coordinates from the camera are transformed to the arm's base coordinate system:

```
P = T + R × (P_pixel - image_center) × scale_factor
```

### 3. Inverse Kinematics
Trigonometry-based inverse kinematics solves for four joint angles from target position (X, Y, Z). The C++ implementation uses iterative search within joint angle constraints (0°~90°). Compiled as a shared library and called from Python for better performance.

### 4. Communication Protocol
Python host sends JSON commands via serial:

```json
{"servo1":90,"servo2":90,"servo3":90,"servo4":145,"servo5":90,"servo6":180}
```

Arduino parses JSON and controls corresponding servos.

## Build & Run

### Compile C++ Kinematics Library

```bash
cd N100
gcc -shared -o calculate_servo_angles.so -fPIC calculate_servo_angles.cpp -lm
```

### Upload Arduino Firmware

Using PlatformIO:
```bash
cd Arduino
pio run --target upload
```

### Run Main Program

```bash
cd N100
python main.py
```

## Serial Configuration

- Port: `/dev/robotarm` (Linux) or `COMx` (Windows)
- Baudrate: 115200
- Note: On Linux, you may need to remove brltty or create a symlink

## Demo Video

[Bilibili](https://www.bilibili.com/video/BV1uHrmYbEYr)

## Parts List

| Part | Qty |
|------|-----|
| MG90S servo | 3 |
| MG996 servo (180°) | 3 |
| Arduino UNO | 1 |
| 5V 3A power adapter | 1 |
| Servo extension cables | as needed |
| M3 screws, nuts, washers | as needed |
