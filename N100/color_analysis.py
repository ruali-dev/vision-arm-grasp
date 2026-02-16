import cv2
import numpy as np

#把摄像头的盖子，打开！

# Trackbar回调函数，这里为空函数
def nothing(x):
    pass

# 创建Trackbar窗口
cv2.namedWindow('Trackbars')
cv2.resizeWindow('Trackbars', 600, 300)

# 创建HSV范围的Trackbar
cv2.createTrackbar('H_min', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('H_max', 'Trackbars', 180, 180, nothing)
cv2.createTrackbar('S_min', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('S_max', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('V_min', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('V_max', 'Trackbars', 255, 255, nothing)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头无法成功读取")
        break
    flipped_frame = cv2.flip(frame, 0)
    # 转换到HSV空间
    hsv = cv2.cvtColor(flipped_frame, cv2.COLOR_BGR2HSV)

    # 获取Trackbar的值
    h_min = cv2.getTrackbarPos('H_min', 'Trackbars')
    h_max = cv2.getTrackbarPos('H_max', 'Trackbars')
    s_min = cv2.getTrackbarPos('S_min', 'Trackbars')
    s_max = cv2.getTrackbarPos('S_max', 'Trackbars')
    v_min = cv2.getTrackbarPos('V_min', 'Trackbars')
    v_max = cv2.getTrackbarPos('V_max', 'Trackbars')

    # 定义阈值范围
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # 生成掩膜
    mask = cv2.inRange(hsv, lower, upper)

    # 连通域分析
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    # 遍历所有连通域（跳过背景，背景的标签是0）
    for i in range(1, num_labels):
        x, y, w, h, area = stats[i]
        center_x, center_y = centroids[i]

        # 过滤掉太小的连通域（噪声）
        if area < 100:
            continue

        # 绘制矩形框
        cv2.rectangle(flipped_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 绘制中心点
        cv2.circle(flipped_frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)

        # 打印中心坐标
        print(f"连通域 {i} 的中心坐标: ({int(center_x)}, {int(center_y)})")

    # 显示图像
    cv2.imshow('Camera feedback', flipped_frame)
    cv2.imshow('Mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()