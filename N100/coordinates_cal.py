import numpy as np

# 1. 标定参数：像素尺寸和图像中心
scale_factor = 0.0480  # 每像素的实际尺寸 (单位：cm/pixel)
image_center = np.array([320, 240])  # 图像中心点 (单位：像素)

# 2. 坐标变换参数
theta = np.radians(-90)  # 旋转角度 (单位：弧度)，逆时针为正
T = np.array([-2, 30.5])    # 平移向量 T = (T_x, T_y) (单位：cm)

# 3. 输入像素坐标
u, v = 294, 200  # 输入点在图像中的像素坐标 (u, v)

# 4. 从像素坐标转换到相机坐标系
P_pixel = np.array([u, v])  # 像素坐标
P_prime = (P_pixel - image_center) * scale_factor  # 转换为相机坐标系 (单位：cm)

# 5. 计算旋转矩阵
R = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)]
])

# 6. 从相机坐标系转换到机械臂基座坐标系
P = T + R @ P_prime  # 坐标变换公式

# 7. 输出结果
print(f"像素坐标 (u, v) = ({u}, {v}) 转换为机械臂坐标系 (X, Y) = ({P[0]:.2f}, {P[1]-15:.2f}) cm")

