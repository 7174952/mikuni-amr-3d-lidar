#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ROSノードの初期化
rospy.init_node('realsense_yolo_node', anonymous=True)
detection_pub = rospy.Publisher('/person_detection_info', String, queue_size=10)
image_pub = rospy.Publisher('/color_image', Image, queue_size=10)
bridge = CvBridge()

# Intel RealSenseカメラの初期化
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 深度ストリームの有効化
pipeline.start(config)

# YOLOモデルの初期化（YOLOv8を使用）
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # 'yolov5m'、'yolov5l'などに置き換えることが可能

# CUDAの有効化（利用可能な場合）
if torch.cuda.is_available():
    model.cuda()

try:
    rate = rospy.Rate(10)  # トピックの発行頻度を10Hzに設定
    while not rospy.is_shutdown():
        # RealSenseカメラのフレームを取得
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()  # 深度フレームを取得
        if not color_frame or not depth_frame:
            continue

        # NumPy配列に変換し画像処理を行う
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())


        # YOLOモデルを使用して人物検出を行う
        results = model(color_image)

        # 検出された人数をカウントし距離情報を取得
        person_count = 0
        detection_info = []
        for *box, conf, cls in results.xyxy[0]:
            if model.names[int(cls)] == 'person':
                person_count += 1
                x1, y1, x2, y2 = map(int, box)
                label = f'{model.names[int(cls)]} {conf:.2f}'
                color = (0, 255, 0)  # 人物を緑色でマーク
                cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # 対象の中心点の深度情報を取得
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                distance = depth_frame.get_distance(center_x, center_y)
                detection_info.append(f'Person {person_count}: Distance = {distance:.2f} meters')

        # 検出された人数と距離情報を出力および発行
        detection_message = f'Detected {person_count} person(s)! ' + '; '.join(detection_info)
        rospy.loginfo(detection_message)
        detection_pub.publish(detection_message)

        # カラー画像を発行
        image_message = bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        image_pub.publish(image_message)

        rate.sleep()  # 10Hzの頻度を維持するために待機
finally:
    # パイプラインを停止しリソースを解放
    pipeline.stop()
    cv2.destroyAllWindows()
