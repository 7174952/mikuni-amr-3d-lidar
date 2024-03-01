#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

rospy.init_node('motor_command_publisher')

pub = rospy.Publisher('/motor_command', Float32MultiArray, queue_size=10)

msg = Float32MultiArray()
msg.layout.dim.append(MultiArrayDimension())
msg.layout.dim[0].label = ''
msg.layout.dim[0].size = 0
msg.layout.dim[0].stride = 0
msg.layout.data_offset = 0
msg.data = [0,0]

rate = rospy.Rate(10) # 10hz

rospy.sleep(1)

for i in range(5):
    pub.publish(msg)
    rate.sleep()

rospy.signal_shutdown('Message published once')