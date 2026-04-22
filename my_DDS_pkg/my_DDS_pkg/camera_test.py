#!/usr/bin/env python3
"""
摄像头参数批量抓拍（无 GUI 版本）
用法:python camera_test.py [camera_index]
抓拍图像保存在 captures/ 目录，文件名包含参数值
"""

import cv2
import numpy as np
import os
import datetime
import sys
import time

# 摄像头索引
camera_index = 0
if len(sys.argv) > 1:
    camera_index = int(sys.argv[1])

# 定义要测试的参数列表，每个参数是一个元组 (属性名, 测试值列表)
# 这里可以根据需要修改或扩展
test_params = {
    'BRIGHTNESS': [40, 50, 60, 70, 80],
    'GAIN': [0, 10, 20, 30],
    'EXPOSURE': [100, 150, 200, 250],   # 注意范围可能不同，根据实际调整
    'AUTO_EXPOSURE': [0, 1],            # 0=手动, 1=自动
    # 'CONTRAST': [40, 50, 60],
    # 'SATURATION': [40, 50, 60],
}

# 固定参数（每次抓拍前会设置的通用参数）
fixed_params = {
    'FRAME_WIDTH': 1280,
    'FRAME_HEIGHT': 720,
    'FPS': 30,
}

# 保存目录
save_dir = "captures_nogui"
os.makedirs(save_dir, exist_ok=True)

def set_camera_prop(cam, prop_name, value):
    """设置摄像头属性，并返回实际值"""
    prop = getattr(cv2, f'CAP_PROP_{prop_name}')
    cam.set(prop, value)
    actual = cam.get(prop)
    print(f"  {prop_name} = {value} (实际 {actual:.1f})")
    return actual

def capture_one(cam, param_name, param_value):
    """抓拍一张图像并保存，文件名包含参数信息"""
    ret, frame = cam.read()
    if not ret:
        print("读取帧失败")
        return False

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    # 文件名包含参数名和值
    filename = os.path.join(save_dir, f"{param_name}_{param_value}_{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    print(f"  保存图像: {filename}")
    return True

def main():
    print(f"打开摄像头 {camera_index} ...")
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("无法打开摄像头")
        sys.exit(1)

    # 设置固定参数（分辨率等）
    for name, val in fixed_params.items():
        set_camera_prop(cap, name, val)

    # 先抓取一张参考图（默认参数）
    print("\n抓取参考图像(默认参数)...")
    capture_one(cap, "default", 0)

    # 遍历每个参数进行测试
    for param_name, test_values in test_params.items():
        print(f"\n测试参数: {param_name}, 测试值: {test_values}")
        for val in test_values:
            # 设置当前参数
            prop = getattr(cv2, f'CAP_PROP_{param_name}')
            cap.set(prop, val)
            actual = cap.get(prop)
            print(f"  设置 {param_name} = {val} (实际 {actual:.1f})")

            # 等待摄像头稳定（某些参数需要几帧生效）
            for _ in range(5):
                cap.read()  # 丢弃前几帧
            time.sleep(0.1)

            # 抓拍
            capture_one(cap, param_name, val)

        # 测试完一个参数后，恢复默认值（可选）
        # 这里可以恢复，但为了简单，不恢复，连续测试

    # 释放摄像头
    cap.release()
    print(f"\n所有测试完成,图像保存在 {save_dir}/")

if __name__ == "__main__":
    main()