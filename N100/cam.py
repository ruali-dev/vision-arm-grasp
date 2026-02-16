import cv2
import numpy as np

# 定义颜色阈值
colors = {
    'blue': {'lower': np.array([100, 50, 50]), 'upper': np.array([130, 255, 255])},
    'green': {'lower': np.array([35, 50, 50]), 'upper': np.array([85, 255, 255])},
    'red': [{'lower': np.array([0, 50, 50]), 'upper': np.array([10, 255, 255])},
            {'lower': np.array([170, 50, 50]), 'upper': np.array([180, 255, 255])}],
    'white':{'lower':np.array([105,0,199]),'upper':np.array([180,255,255])}
}

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头无法成功读取")
        break
    flipped_frame = cv2.flip(frame, 0)
    # 转换到HSV空间
    hsv = cv2.cvtColor(flipped_frame, cv2.COLOR_BGR2HSV)

    # 处理每个颜色
    for color, thresholds in colors.items():
        if color == 'red':
            # 红色有两部分
            mask_red1 = cv2.inRange(hsv, thresholds[0]['lower'], thresholds[0]['upper'])
            mask_red2 = cv2.inRange(hsv, thresholds[1]['lower'], thresholds[1]['upper'])
            mask_color = cv2.bitwise_or(mask_red1, mask_red2)
        else:
            mask_color = cv2.inRange(hsv, thresholds['lower'], thresholds['upper'])
        
        # 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_color, connectivity=8)
        
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
            print(f"{color} 连通域 {i} 的中心坐标: ({int(center_x)}, {int(center_y)})")

    # 显示图像
    cv2.imshow('Camera feedback', flipped_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()