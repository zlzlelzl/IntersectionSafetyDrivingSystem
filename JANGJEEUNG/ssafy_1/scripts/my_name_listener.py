#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from ssafy_1.msg import student

# my_name_listener 는 Custom Msgs 를 이용한 Topic Subscriber(메세지 수신) 예제입니다.
# /my_name 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def callback(data):
    rospy.loginfo('\n my name : %s %s \n my age : %i \n SSAFY score : %i', data.first_name,data.last_name,data.age,data.score)

def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('my_name_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    rospy.Subscriber('my_name', student, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
