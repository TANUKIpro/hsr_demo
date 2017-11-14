#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Twist
import tf2_ros
import tf2_geometry_msgs

y = 0
def callback_arata(data):
    global y
    y = data.wrench.torque.y
    
sub = rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped, callback_arata, queue_size=1)
rospy.init_node('hsnd_starter')

while not rospy.is_shutdown():
    if y < -2.0:
        print('break')
        break
    rospy.sleep(0.1)
    pass
