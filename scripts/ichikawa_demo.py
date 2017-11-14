#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import math
import rospy
from roslib import message
from geometry_msgs.msg import Twist
from math import copysign
from math import pi
from sound_play.libsoundplay import SoundClient
import sys
from std_msgs.msg import *
import smach
import smach_ros
import sensor_msgs
import sensor_msgs.msg
#import hokuyo_node
import math
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from geometry_msgs.msg import *
import datetime
from collections import OrderedDict
import numpy as np
import hsrb_interface
from hsrb_interface import geometry

import rospy
import tf2_ros
import math

from geometry_msgs.msg import PoseStamped, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_geometry_msgs
from sensor_msgs.msg import Image , CameraInfo
import message_filters
from yolo_tf.msg import ObjectArray

import hsrb_interface
from hsrb_interface.exceptions import MobileBaseError
import rospy
import sys
from hsrb_interface import geometry, Robot
import message_filters
from sensor_msgs.msg import Image , CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
from yolo_tf.msg import ObjectArray
import numpy as np
import image_geometry
import tf2_ros
from geometry_msgs.msg import PointStamped , Twist
import tf2_geometry_msgs
import controller_manager_msgs.srv
import roslib
import math
import threading
import yolo_tf.libs as yl

import rospy
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Twist
import tf2_ros
import tf2_geometry_msgs

y = 0

def callback_arata(data):
    global y
    y = data.wrench.torque.y
    
sub = rospy.Subscriber('/hsrb/wrist_wrench/compensated', WrenchStamped, callback_arata, queue_size=1)
#rospy.init_node('oppai')


robot = Robot()
 
whole_body = robot.try_get('whole_body')
omni_base = robot.try_get('omni_base')
collision_world = robot.try_get('global_collision_world')
suction = robot.try_get('suction')
gripper = robot.try_get('gripper')
wrist_wrench = robot.try_get('wrist_wrench')
marker = robot.try_get('marker')
battery = robot.try_get('battery')
tts = robot.try_get('default_tts')

whole_body.move_to_neutral()
gripper.command(1)

print "start"


tr = yl.TreeReader()

class demo():
  def __init__(self):
    print "move"
    tts.say("go to table")
    omni_base.go(0.18838588270200388, 1.6078506143263152, -3.0846642639937287,20,relative=False)
    whole_body.move_to_neutral()
    
    self.flag = False
    self.bridge = CvBridge()
    self.pinhole= image_geometry.PinholeCameraModel()
    print 1
    self._tf_buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._tf_buffer)
    self.pinhole_info = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/camera_info' , CameraInfo , self.camera, queue_size = 1)

    print 2
    objsub = message_filters.Subscriber('/objects', ObjectArray)
    depthsub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image', Image)
    image_person = message_filters.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image) 
    ts = message_filters.ApproximateTimeSynchronizer([objsub, depthsub , image_person], 100 , 0.3)

    print 3
    ts.registerCallback(self.find_person)
    print "init finish"

  def camera(self , data):
        self.pinhole.fromCameraInfo(data)

  def find_person(self, object, depth, image):
        #print 'callback'
        result_array = []
        print "sub"
        for pobj in object.objects:
           if (self.flag):
               break
           person_probability = tr.probability(pobj.class_probability, "smelling bottle")
           person_probability *= pobj.objectness
           if person_probability < 0.001:
               continue
           print "OK"
           top = pobj.top
           left = pobj.left
           bottom = pobj.bottom
           right = pobj.right
           change_dep = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
           self.depth = change_dep
           self.person_follow(top , bottom , right , left , self.depth , object)

  def person_follow(self , top , bottom , right , left , depth , objarr):
         y = int((top + bottom)/2)
         x = int((left + right)/2)
         ray = self.pinhole.projectPixelTo3dRay([x , y])
         camera_point = np.array(ray) * depth[y , x]

         pointstamped = PointStamped()

         pointstamped.header.stamp = rospy.Time.now()
         pointstamped.header.frame_id = objarr.header.frame_id
         pointstamped.point.x = camera_point[0]
         pointstamped.point.y = camera_point[1]
         pointstamped.point.z = camera_point[2]

         try:
             pointstamped = self._tf_buffer.transform(pointstamped , "base_footprint" , timeout = rospy.Duration(5))
             print pointstamped.point
             tts.say("try to take the bottle")
             whole_body.move_end_effector_pose(((pointstamped.point.x+0.05,pointstamped.point.y,pointstamped.point.z),(math.sqrt(2)/2,0,math.sqrt(2)/2,0)))
             gripper.grasp(-0.01)
             omni_base.go(-0.4,0,0,10,relative=True)
             whole_body.move_to_go()
             omni_base.go(1.4842096504348206, 1.7556134149575287, 1.568438660836788,20,relative=False)
             whole_body.move_to_neutral()
             gripper.command(1)
             tts.say(u'hoi')
             self.flag = True
         except:
             print "oppai"
             
def main():
  #say.tts('push my hand')
  while not rospy.is_shutdown():
    if y < -2.0:
      print('break')
      break
    rospy.sleep(0.1)
    pass
  print "node"
  demo()
         
if __name__ == '__main__':
  while not rospy.is_shutdown():
    main()
  rospy.spin()
