#%%
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from math import cos, sin, atan2, sqrt, pow, pi
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList,GetTrafficLightStatus
from lib.mgeo.class_defs import *
from collections import deque

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
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_light_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
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
        self.r = float('inf')
        
        # 정지선 리스트
        current_path = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(current_path)
        
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        lane_boundary_set = mgeo_planner_map.lane_boundary_set
        self.lanes = lane_boundary_set.lanes

        # 정지선 변수
        self.stopped_time = 0
        self.ignore_stoplanes = deque()

        # 횡단보도 리스트
        cw_set = mgeo_planner_map.cw_set
        self.cws = cw_set.data
        # print(len(self.cws))
        scw_set = mgeo_planner_map.scw_set
        self.scws = scw_set.data
        # print(len(self.scws))
        

        # mgeo에서 신호,교차로 정보 받아오기
        self.is_traffic_light = False 
        self.traffic_set = mgeo_planner_map.light_set
        self.intersection_set = mgeo_planner_map.intersection_controller_set
        self.intersection_state_set = mgeo_planner_map.intersection_state_list

        self.stoplanes = []
        for lane in self.lanes:
            # 점선은 525 정지선은 530
            if 530 in self.lanes[lane].lane_type:
                self.stoplanes.append(lane)

        # 교차로 boundary 설정
        intersection_points = []
        import time
        S = time.process_time()
        count = 0
        bbox = []
        for k,v in self.intersection_set.intersection_controllers.items():
            min_x, min_y, max_x, max_y = 9999,9999,-9999,-9999
            #print(k)
            #print(list(v.get_signal_list()))
            for sig in list(v.get_signal_list()):
                #print(sig.ref_crosswalk_id)
                for scws_idx,value in self.scws.items():
                    # count += 1
                    xx = value.bbox_x
                    yy = value.bbox_y
                    for x in xx:
                        for y in yy:
                            bbox.append([x,y])

                    #print(value.ref_crosswalk_id)
                    if value.ref_crosswalk_id == sig.ref_crosswalk_id:
                        # print(value.ref_crosswalk_id)
                        # print(value.points)
                        for i in range(0,5):
                            min_x = min(min_x,value.points[i][0])
                            min_y = min(min_y,value.points[i][1])
                            max_x = max(max_x,value.points[i][0])
                            max_y = max(max_y,value.points[i][1])
            intersection_points.append([min_x,min_y,max_x,max_y])
        E = time.process_time()
        # print("time = ", E-S)
        #print(intersection_points)
    
        print(bbox)
        rate = rospy.Rate(20) # 20hz.
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

                # print(x,y)
                
                # import json
                # data = ""
                # print("[")
                # for p in self.local_path_msg.poses:
                #     pose = p.pose.position
                #     print("[{0},{1}]".format(pose.x, pose.y))
                # print("]")
                    
                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(self.local_path_msg)
                
                velocity_msg = Float32()
                velocity_msg = self.find_target_velocity()

                # 정지선을 매번 체크하는 것보다 우회전하기 위해 충분히 속도를 줄였을 때 정지선을 체크하는게 최적화에 도움됨
                # 우회전 조건식 넣어주기
                
                # local path 1 0 = 직진, 1 = 좌회전 2= 우회전
                path_status = -1

                # 교차로 진입 확인
                is_intersection = False
                for int_idx,int_point in enumerate(intersection_points):
                    for lpath_idx in range(len(self.local_path_msg.poses)-1, -1, -1):
                        end_point = self.local_path_msg.poses[lpath_idx].pose.position
                        if int_point[2] > end_point.x > int_point[0] and int_point[3] > end_point.y > int_point[1]:
                            #now_intersection_idx = int_idx
                            is_intersection = True
                            break
                    if is_intersection:
                        break
                
                

                # 신호 감지
                if self.is_traffic_light:
                    #print("신호감지")
                    now_traffic_light_idx = self.traffic_light_data.trafficLightIndex
                    now_traffic_color = self.traffic_light_data.trafficLightStatus
                    print(f'신호등 인덱스 = {now_traffic_light_idx}, 신호 = {now_traffic_color}')
                    # 빨간불 = 1, 노란색 = 4 직진 = 16 좌회전 = 32

                    # 정지선 인지, 거리 계산
                    stop_lane_pos = self.find_stop_lane_in_local_path()

                    #정지 (빨 + 빨+좌)
                    if now_traffic_color % 2 == 1:

                        if len(stop_lane_pos) != 0:
                            #dis_traffic_light = stop_lane_pos[2]
                            velocity_msg = self.find_target_velocity_stoplane(stop_lane_pos)
                            # # 제동 거리
                            # dis_safe = pow(self.status_msg.velocity.x,2) / (2 * 9.8 * self.friction)
                            # #print(f'제동거리 = {dis_safe+5}, 정지선거리 = {dis_traffic_light}')
                            # if dis_traffic_light < dis_safe + 10:
                            #     velocity_msg = 0

                    #직진 (직, 직+좌)
                    elif now_traffic_color & 16 == 0:
                        pass

                    # 교차로 진입 할때
                    if is_intersection:
                        print("교차로")
                        #path_status 판단
                        lps_s_point = self.local_path_msg.poses[0].pose.position
                        lps_e_point = self.local_path_msg.poses[99].pose.position

                        degree = self.status_msg.heading - atan2(lps_e_point.y - lps_s_point.y , lps_e_point.x - lps_s_point.x)*(180 / pi)
                        #slope = lps_e_point.x - lps_s_point.x / lps_e_point.y - lps_s_point.y

                        print(f'곡률 : {self.r}')
                        #print(f'기울기 : {slope}')
                        print(f'degree = {degree}')

                        if -30 < degree < 30:
                            path_status = 0
                        elif 30< degree < 60:
                            path_status = 2
                        elif -30> degree > -60:
                            path_status = 1

                        if path_status == 1:  # 좌회전

                            #정지 (빨간불)
                            if now_traffic_color % 2 == 1:

                                if len(stop_lane_pos) != 0:
                                    dis_traffic_light = stop_lane_pos[2]
                                    velocity_msg = self.find_target_velocity_stoplane(stop_lane_pos)
                                    # # 제동 거리
                                    # dis_safe = pow(self.status_msg.velocity.x,2) / (9.8 * self.friction)
                                    # #print(f'제동거리 = {dis_safe+5}, 정지선거리 = {dis_traffic_light}')
                                    # if dis_traffic_light < dis_safe + 2:
                                    #     velocity_msg = 0
                            #직진 (직, 직+좌)
                            elif now_traffic_color & 16 == 0:
                                pass

                        elif path_status == 2:  # 우회전

                            if len(stop_lane_pos) != 0:
                                velocity_msg = self.find_target_velocity_stoplane(stop_lane_pos)
                                if velocity_msg <= 2:
                                    self.stopped_time += 1
                                    if self.stopped_time >= 20:
                                        if len(self.ignore_stoplanes) > 10:
                                            self.ignore_stoplanes.pop()
                                        self.ignore_stoplanes.appendleft([stop_lane_pos[0], stop_lane_pos[1]])
                                        self.stopped_time = 0
                                    # print(self.ignore_stoplanes, self.stopped_time)
                                else:
                                    self.stopped_time = 0
                print(path_status)
                #print(f'target_v = {velocity_msg}')
                self.velocity_pub.publish(velocity_msg)
                self.is_traffic_light = False

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg

    def traffic_light_callback(self,data):
        self.traffic_light_data = data
        self.is_traffic_light = True

    def status_callback(self, msg):
        self.status_msg = msg
            
    def find_target_velocity_stoplane(self, stop_lane_pos):
        # 아마 직진거리로 계산할 경우 곡선 형태의 감속 구간에서 언더스티어날듯
        # 노드마다 거리합으로 바꿔줘야됨
        # distance=sqrt(pow(self.x-stop_lane_pos[0],2)+pow(self.y-stop_lane_pos[1],2))
        distance = stop_lane_pos[2]
        # 감속 최대(10m/s^2)
        print(f'정지선 까지 거리 = {distance}')
        # v^2 = u^2 + 2as
        velocity = Float32()
        velocity = max(0, sqrt(2 * 10 * distance) - 12)
        # print(velocity)
        return velocity
    
    # def find_stop_lane_in_local_path(self):
    #     curve_distance=0
    #     prev_x, prev_y = self.x, self.y
    #     for p in self.local_path_msg.poses:                    
    #         curve_distance+=sqrt(pow(prev_x-p.pose.position.x,2)+pow(prev_y-p.pose.position.y,2))
    #         prev_x, prev_y = p.pose.position.x, p.pose.position.y
    #         for stoplane in self.stoplanes:
    #             points = self.lanes[stoplane].points
    #             for point in points:
    #                 x, y = point[0], point[1]
    #                 distance=sqrt(pow(x-p.pose.position.x,2)+pow(y-p.pose.position.y,2))
    #                 if distance < 0.5:
    #                     return [x, y, curve_distance]
    #     return []

    def find_stop_lane_in_local_path(self):
        curve_distance=0
        prev_x, prev_y = self.x, self.y
        for p in self.local_path_msg.poses:
            curve_distance+=sqrt(pow(prev_x-p.pose.position.x,2)+pow(prev_y-p.pose.position.y,2))
            prev_x, prev_y = p.pose.position.x, p.pose.position.y
            #print(self.ignore_stoplanes)
            for stoplane in self.stoplanes:
                points = self.lanes[stoplane].points
                
                ignore_flag = False
                for point in points:
                    
                    
                    x, y = point[0], point[1]
                    if len(self.ignore_stoplanes) > 0:
                        ignore_x, ignore_y = self.ignore_stoplanes[0][0], self.ignore_stoplanes[0][1]
                        
                        if x == ignore_x and y == ignore_y:
                            ignore_flag = True
                            continue
                    
                    distance=sqrt(pow(x-p.pose.position.x,2)+pow(y-p.pose.position.y,2))
                    if distance < 0.3:
                        return [x, y, curve_distance]
                
                    
        return []
    
    def find_target_velocity(self):
        self.r = self.find_r()
        velocity = Float32()
        velocity = sqrt(abs(self.r) * 9.8 * self.friction)
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
# %%
__file__