#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

def publish_initial_pose():
    rospy.init_node("initial_pose_publisher", anonymous=True)
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

    # 等待一下，确保有订阅者（如 hdl_localization）建立连接
    rospy.sleep(1.0)

    # 构造初始位姿消息
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    # 设定位置
    pose_msg.pose.pose.position.x = rospy.get_param("~init_pos_x", 0.0)
    pose_msg.pose.pose.position.y = rospy.get_param("~init_pos_y", 0.0)
    pose_msg.pose.pose.position.z = rospy.get_param("~init_pos_z", 0.0)

    # 设定方向 (四元数)
    pose_msg.pose.pose.orientation.x = rospy.get_param("~init_ori_x", 0.0)
    pose_msg.pose.pose.orientation.y = rospy.get_param("~init_ori_y", 0.0)
    pose_msg.pose.pose.orientation.z = rospy.get_param("~init_ori_z", 0.0)
    pose_msg.pose.pose.orientation.w = rospy.get_param("~init_ori_w", 1.0)

    # 设定初始协方差 (可根据实际需要调整)
    pose_msg.pose.covariance = [
        0.0, 0,   0,   0,   0,   0,
        0,   0.0, 0,   0,   0,   0,
        0,   0,   0.0, 0,   0,   0,
        0,   0,   0,   0.0, 0,   0,
        0,   0,   0,   0,   0.0, 0,
        0,   0,   0,   0,   0,   0.0
    ]

    # 发布消息
    pub.publish(pose_msg)
    rospy.loginfo("Published initial pose to /initialpose")
    time.sleep(1.0)
    pub.publish(pose_msg)
    rospy.loginfo("Published initial pose to /initialpose")

if __name__ == "__main__":
    publish_initial_pose()

