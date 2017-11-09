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
   (u'それじゃあロボット工房のデモプログラムを開始するね！'),]

NAVI_SENARIO = [
   ((1.60, 1.73, 1.57, 180.0), u'ここは一般の家庭のリビングを想定した場所だよ。テーブルやソファー、冷蔵庫があるのはそれが理由なんだ。'),#客の前
   ((-0.56, -1.268, 3.14, 180.0), u'ここはヒロをコントロールするパソコンが置いてあるよ！今、ヒロが動いてるのはここでスクリプトを書いたおかげなんだ！'),#八木_wsのところ
   ((3.39, -1.4545, -1.57, 180.0), u'ここは本棚。難しい本がたくさん並んでいるね！僕にはわかんないや！'),#本棚のところ
   ((2.366, 1.081, -1.57, 180.0), u'ここはガラスのテーブル。普段のロボット工房でここに来ると、頭を抱えている人に会えるかも！？')]#テーブルの前

END_TALK_SENARIO = [
   (u'')
   (u'ここからは')
   (u'')
   (u'')
   (u'')]

class Demo():
    def say_and_sleep(self, contents=''):
        tts.say(contents)
        rospy.sleep(len(contents)*0.23)
        
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
        except:
            rospy.logerr('Fail intro')
            tts.say(u'ほえーーー！introは失敗')
            
            self.status = Faild
            return self.status
            
    def what_can_i_do(self):
        try:
            tts.say(u'まず僕の一通りの機能を紹介するよ')
            rospy.sleep(5)
        except:
            rospy.logerr('Fail what_can_i_do')
            
    def navi(self, pos=(0, 0, 0), contents=''):
        try:
            omni_base.go_abs(pos[0], pos[1], pos[2], 180.0)
        except:
            rospy.logerr('Fail navi')
        tts.say(contents)
        rospy.sleep(5)
        
    def end_talk(self):
        pass
        
def main():
    demo = Demo()
    
    demo.intro()
    rospy.sleep(3)
    
    for unit in INTRO_SENARIO:
        demo.say_and_sleep(unit)
    rospy.sleep(5)
    
    demo.what_can_i_do()
    rospy.sleep(5)
    
    for unit in NAVI_SENARIO:
        demo.navi(unit[0], unit[1])
    rospy.sleep(5)
    
if __name__=='__main__':
    try:
        whole_body.move_to_go()
    except:
        rospy.logerr('Fail move_to_neutral')
    main()
