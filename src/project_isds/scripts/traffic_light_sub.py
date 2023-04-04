#!/usr/bin/env python3.7
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from morai_msgs.msg import TrafficSignInfo

# my_name_listener 는 Custom Msgs 를 이용한 Topic Subscriber(메세지 수신) 예제입니다.
# /my_name 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력
def callback(data):
    for i in range(len(data.traffic_light)):
        d = data.traffic_light[i]
        sign_type = d.traffic_light_type
        sign_status = d.traffic_light_status
        print('#'+str(i), sign_type, sign_status)

    print()
    
def listener():
    #TODO: (1) ROS 노드 이름 선언
    rospy.init_node('topic_listener', anonymous=True)

    #TODO: (2) Subscriber 생성
    
    # student 라는 직접 만든 Custom ROS 메세지 형식을 사용하여 Topic Subscriber 를 완성한다.
    rospy.Subscriber( 'traffic_sign_info' , TrafficSignInfo , callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    listener()
