#!/usr/bin/env python
#-*- coding:utf-8 -*-

import sys, os
import rospy
from hsrb_interface import Robot

robot = Robot()
omni_base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')

INTRO_SENARIO = [
   (u'こんにちは！僕はトヨタで作られた、暮らしをサポートするパートナーロボット。HSRです。'),
   (u'今日はロボット工房に足を運んでくれてありがとう！'),
   (u'久しぶりのお客さんなので、少し緊張ぎみかも。失敗したら、ごめんね？'),
   (u'それじゃあロボット工房のデモプログラムを開始するよ！'),]

NAVI_SENARIO = [
   ((1.60, 1.73, 1.57, 180.0), u'ここは一般の家庭のリビングを想定した場所だよ。テーブルやソファー、冷蔵庫があるのはそれが理由なんだ。'),#客の前
   ((-0.56, -1.268, 3.14, 180.0), u'ここはヒロをコントロールするパソコンが置いてあるよ！今、ヒロが動いてるのはここでスクリプトを書いたおかげなんだ！'),#八木_wsのところ
   ((3.39, -1.4545, 1.57, 180.0), u'ここは本棚。難しい本がたくさん並んでいるね！僕にはわかんないや！'),#本棚のところ
   ((2.366, 1.081, -1.57, 180.0), u'ここはガラスのテーブルとソファー。普段のロボット工房でここに来ると、頭を抱えている人に会えるかも！？')]#テーブルの前

END_TALK_SENARIO = [
   (u'以上でロボット工房のデモプログラムを終了します。'),
   (u'本当はもっと沢山のことができるけど、このプログラムを作った人にはこれが限界みたいだね！'),
   (u'もしかすると、この後説明があるかもしれないけど、今年のロボカップ世界大会では僕達のチームが準優勝を獲得したんだ。'),
   (u'その時は自分でドアを開けてテーブルにあるオブジェクトを棚に並べていったり、人の後をついていったりしたんだ。'),
   (u'話が長くなっちゃったね。今日は本当にありがとう。'),
   (u'最後に。現在ロボット工房では、僕達と一緒にHSRの研究をしてくれるメンバーを募集中です。興味がある方はお近くのスタッフまでー！')]

class Demo():
    def say_and_sleep(self, contents=''):
        tts.say(contents)
        rospy.sleep(len(contents)*0.23)
        rospy.logerr('callback')
        
    def intro(self):
        self.status = None
        try:
            omni_base.go_abs(-0.7, 0, 0, 180.0)     #front of the door point
            tts.language = tts.ENGLISH
            tts.say(u'Demo start')
            rospy.sleep(3)
            omni_base.go_abs(1.60, 1.73, 1.57, 180.0)  #front of people
            tts.language = tts.JAPANESE
            whole_body.move_to_joint_positions({"head_tilt_joint":0.2})      #首を上げる
            rospy.logerr('start intro')
        except:
            rospy.logerr('Fail intro')
            tts.say(u'ほえーーー！introは失敗')
            
            self.status = Faild
            return self.status
            
    def what_can_i_do(self):
        try:
            tts.say(u'まず僕の一通りの機能を紹介するよ')
            rospy.sleep(5)
            tts.say(u'まずは僕の足！先輩のASHIMOさんみたいに素敵な足はついてないけど、かわりにオムニホイールっていう、その場から全方向に移動可能な足がついてるんだ')
            rospy.sleep(45)
            tts.say(u'こんな風にその場で回ったり')
            omni_base.go_abs(1.60, 1.73, -1.57, 180.0)
            rospy.sleep(1)
            omni_base.go_abs(1.60, 1.73, 1.57, 180.0)
            rospy.sleep(1)
            
            tts.say(u'左右に移動できるんだ')
            omni_base.go_abs(1.60+0.7, 1.73, 1.57, 10.0)
            omni_base.go_abs(1.60-0.7, 1.73, 1.57, 10.0)
            
            omni_base.go_abs(1.60, 1.73, 1.57, 180.0)
            rospy.sleep(1)
            tts.say(u'それとこのマニュピレーターにも注目してほしいな！')
            rospy.sleep(5)
            
            tts.say(u'説明するもの退屈だから、今から、そこにあるペットボトルをとってみせるね！')
            omni_base.go_abs(1.60, 1.73, 3.14, 180.0)
            rospy.logerr('success what_can_i_do')
            rospy.sleep(20)
            
            
        except:
            rospy.logerr('Fail what_can_i_do')
            tts.language = tts.ENGLISH
            tts.say('faild what can i do')
            tts.language = tts.JAPANESE
            omni_base.go_abs(0, 0, 0, 180.0)
            
    def navi(self, pos=(0, 0, 0), contents=''):
        try:
            omni_base.go_abs(pos[0], pos[1], pos[2], 180.0)
            rospy.logerr('navi')
        except:
            rospy.logerr('Fail navi')
            tts.language = tts.ENGLISH
            tts.say('faild navigation')
            tts.language = tts.JAPANESE
            omni_base.go_abs(0, 0, 0, 180.0)
        tts.say(contents)
        rospy.sleep(7)
        
    def end_talk(self):
        try:
            omni_base.go_abs(1.60, 1.73, 1.57, 180.0)
            whole_body.move_to_joint_positions({"head_tilt_joint":0.2})
            rospy.logerr('success end_talk')
        except:
            rospy.logerr('Fail end_talk')
            tts.language = tts.ENGLISH
            tts.say('faild what can i do')
            tts.language = tts.JAPANESE
            omni_base.go_abs(0, 0, 0, 180.0)
            
class Grep_the_bottle():
    def __init__(self):
        tts.language = tts.ENGLISH
        tts.say('go to table')
        omni_base.go(0.18838, 1.60785, -3.08466,20,relative=False)
        
        self.bridge = CvBridge()
        self.pinhole= image_geometry.PinholeCameraModel()
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        self.pinhole_info = rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/camera_info' , CameraInfo , self.camera, queue_size = 1)
        
        objsub = message_filters.Subscriber('/objects', ObjectArray)
        depthsub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image', Image)
        image_person = message_filters.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_color', Image) 
        ts = message_filters.ApproximateTimeSynchronizer([objsub, depthsub , image_person], 100 , 0.3)
        ts.registerCallback(self.find_person)
        
    def camera(self, data):
        self.pinhole.fromCameraInfo(data)
        
    def find_object(self, object, depth, image):
        result_array = []
        for pobj in object.objects:
           if (self.flag):
               break
           person_probability = tr.probability(pobj.class_probability, "smelling bottle")
           person_probability *= pobj.objectness
           if person_probability < 0.001:
               continue
           
           top = pobj.top
           left = pobj.left
           bottom = pobj.bottom
           right = pobj.right
           change_dep = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
           self.depth = change_dep
           self.person_follow(top , bottom , right , left , self.depth , object)
        
    def distance_from_bottle(self , top , bottom , right , left , depth , objarr):
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
            whole_body.move_end_effector_pose(((pointstamped.point.x+0.02,pointstamped.point.y,pointstamped.point.z),(math.sqrt(2)/2,0,math.sqrt(2)/2,0)))
            gripper.grasp(-0.01)
            omni_base.go(-0.4,0,0,10,relative=True)
            whole_body.move_to_go()
            self.flag = True
            omni_base.go(1.48420, 1.75561, 1.56843,20,relative=False)
            whole_body.move_to_neutral()
            gripper.command(1)
        except:
            rospy.logerr('faild')
        
def main():
    demo = Demo()
    
    demo.intro()#intro
    rospy.sleep(3)
    
    for unit in INTRO_SENARIO:
        demo.say_and_sleep(unit)
    print('success intro')
    rospy.sleep(5)
    
    demo.what_can_i_do()#what_can_i_do
    print('success what_can_i_do')
    rospy.sleep(5)
    
    for unit in NAVI_SENARIO:#navi
        demo.navi(unit[0], unit[1])
    print('success navigation')
    rospy.sleep(5)
    
    demo.end_talk()#end_talk
    rospy.sleep(1)
    
    for unit in END_TALK_SENARIO:
        demo.say_and_sleep(unit)
    print('success end_talk')
    rospy.sleep(2)
    
if __name__=='__main__':
    try:
        whole_body.move_to_go()
    except:
        rospy.logerr('Fail move_to_neutral')
    main()
    
    rospy.logerr('Fail what_can_i_do')
    tts.language = tts.ENGLISH
    tts.say('All senario have ended. Move to the origin.')
    rospy.sleep(3)
    tts.language = tts.JAPANESE
    omni_base.go_abs(-0.7, 0, 0, 180.0)
    whole_body.move_to_neutral()
    
    rospy.logerr('Finish')
