#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos, sin, atan2, sqrt, pow, pi
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# local_path_pub 은 global Path (전역경로) 데이터를 받아 Local Path (지역경로) 를 만드는 예제입니다.
# Local Path (지역경로) 는 global Path(전역경로) 에서 차량과 가장 가까운 포인트를 시작으로 만들어 집니다.

# 노드 실행 순서 
# 1. Global Path 와 Odometry 데이터 subscriber 생성 
# 2. Local Path publisher 선언
# 3. Local Path 의 Size 결정
# 4. 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
# 5. Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
# 6. 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리 
# 7. Local Path 메세지 Publish


class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        
        #TODO: (1) Global Path 와 Odometry 데이터 subscriber 생성 
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber('/global_path',Path, self.global_path_callback)

        #TODO: (2) Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)
        self.velocity_pub = rospy.Publisher('/velocity', Float32, queue_size=1)
        
        # 초기화
        self.is_odom = False
        self.is_path = False

        #TODO: (3) Local Path 의 Size 결정
        self.local_path_size = 100          # 50 m
        self.max_velocity = 100.0 / 3.6     # 100 km/h
        self.friction = 0.8

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():

            if self.is_odom == True and self.is_path == True:
                self.local_path_msg=Path()
                self.local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
                min_dis=float('inf')
                current_waypoint=-1
                for i,waypoint in enumerate(self.global_path_msg.poses) :

                    distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                    if distance < min_dis :
                        min_dis=distance
                        current_waypoint=i
                
                #TODO: (6) 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리
                if current_waypoint != -1 :
                    for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                        if num - current_waypoint >= self.local_path_size:
                            break
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1
                        self.local_path_msg.poses.append(tmp_pose)

                velocity_msg = Float32()
                velocity_msg = self.find_target_velocity()
                
                print(x,y)
                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(self.local_path_msg)
                self.velocity_pub.publish(velocity_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg
        
    def find_target_velocity(self):
        r = self.find_r()
        velocity = Float32()
        velocity = sqrt(abs(r) * 9.8 * self.friction)
        if velocity > self.max_velocity:
            velocity = self.max_velocity
        
        return velocity

    def find_r(self):
        
        small_size = 25
        big_size = 99

        big_X_array = []
        big_Y_array = []

        big_r = float('inf')
        small_r = float('inf')
        min_small_r = float('inf')

        for big_idx in range(0,big_size+1,big_size//2):
            x = self.local_path_msg.poses[big_idx].pose.position.x
            y = self.local_path_msg.poses[big_idx].pose.position.y
            big_X_array.append([x,y,1])
            big_Y_array.append([-(x**2)-(y**2)])

        if(np.linalg.det(big_X_array)):
                
            X_inverse = np.linalg.inv(big_X_array)

            A_array = X_inverse.dot(big_Y_array)
            a = A_array[0]*-0.5
            b = A_array[1]*-0.5
            c = A_array[2]
            big_r = sqrt(a*a + b*b - c)
        
        for path_idx in range(0,len(self.local_path_msg.poses) - small_size):
            
            small_X_array = []
            small_Y_array = []

            for small_idx in range(0,small_size+1,small_size//2):
                x = self.local_path_msg.poses[path_idx+small_idx].pose.position.x
                y = self.local_path_msg.poses[path_idx+small_idx].pose.position.y
                small_X_array.append([x,y,1])
                small_Y_array.append([-(x**2)-(y**2)])

            if(np.linalg.det(small_X_array)):
                
                X_inverse = np.linalg.inv(small_X_array)

                A_array = X_inverse.dot(small_Y_array)
                a = A_array[0]*-0.5
                b = A_array[1]*-0.5
                c = A_array[2]

                small_r = sqrt(a*a + b*b - c)
                
            if min_small_r > small_r:
                min_small_r = small_r
        
        r = min(min_small_r,big_r)

        return r

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass