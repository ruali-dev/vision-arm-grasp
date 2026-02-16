import cv2
import numpy as np

# 打开摄像头
cap = cv2.VideoCapture(0)

while True:
    ret, image = cap.read()
    if not ret:
        print("摄像头无法成功读取")
        break

    # 1. 转换为 HSV 色彩空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 2. 设置白色的 HSV 范围
    # 白色范围可以根据环境光调整，以下是一个通用的范围
    lower_white = np.array([109, 0, 223])  # H: 0~180, S: 0~255, V: 200~255
    upper_white = np.array([180, 255, 255])

    # 3. 提取白色区域
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # 4. 形态学操作（可选）——去除噪声
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 5. 轮廓检测
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # 6. 找到最大的轮廓（假设正方体是图像中的最大白色区域）
        max_contour = max(contours, key=cv2.contourArea)

        # 7. 计算轮廓的外接矩形
        x, y, w, h = cv2.boundingRect(max_contour)

        # 在图像上绘制矩形框
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(image, f"{w}x{h} pixels", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 8. 计算每个像素的实际尺寸（假设正方体边长为 3cm）
        cube_real_size = 3.0  # 正方体实际边长（单位：cm）
        scale_factor_x = cube_real_size / w  # X 方向每像素实际长度（单位：cm/pixel）
        scale_factor_y = cube_real_size / h  # Y 方向每像素实际长度（单位：cm/pixel）

        # 9. 打印结果
        print(f"正方体的像素尺寸: 宽度 = {w} 像素, 高度 = {h} 像素")
        print(f"每像素实际尺寸: X方向 = {scale_factor_x:.4f} cm/pixel, Y方向 = {scale_factor_y:.4f} cm/pixel")
    else:
        print("未检测到白色正方体")

    # 10. 显示图像
    cv2.imshow("Detected Cube", image)
    cv2.imshow("Mask", mask)  # 显示二值化后的掩膜图像

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

