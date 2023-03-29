#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import time
from math import cos, sin, atan2, sqrt, pow, pi
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,ObjectStatusList,GetTrafficLightStatus
from lib.mgeo.class_defs import *
from collections import deque
from tf.transformations import euler_from_quaternion

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
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        #TODO: (2) Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)
        self.velocity_pub = rospy.Publisher('/velocity1', Float32, queue_size=1)
        # 정지선 publish
        #self.stoplane_pub = rospy.Publisher('/stoplane', Path, queue_size=1)
        
        # 초기화
        self.is_odom = False
        self.is_path = False
        self.is_status = False
        
        #TODO: (3) Local Path 의 Size 결정
        self.local_path_size = 100          # 50 m
        self.max_velocity = 100.0 / 3.6     # 100 km/h
        self.friction = 0.8
        # self.r = float('inf')
        
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
        scw_set = mgeo_planner_map.scw_set
        self.scws = scw_set.data

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

        # 교차로,횡단보도 boundary 설정
        # intersection_points = []
        intersection_points = dict() # 교차로 boundary
        self.intersection_crosswalk_idx = dict() # 교차로 내부 횡단보도 idx

        for k,v in self.intersection_set.intersection_controllers.items():
            min_x, min_y, max_x, max_y = 9999,9999,-9999,-9999
            self.intersection_crosswalk_idx[k] = []
            prev_cw_name = ""
            # print(k)
            #print(list(v.get_signal_list()))
            for sig in list(v.get_signal_list()):
                #print(sig.ref_crosswalk_id)
                for scws_idx,value in self.scws.items():
                    # print(
                    #     # value.bbox_y,
                    #     #dir(value)
                    #     sig
                    #     )
                    if value.ref_crosswalk_id == sig.ref_crosswalk_id:
                        if prev_cw_name != value.ref_crosswalk_id:
                            self.intersection_crosswalk_idx[k].append(value.ref_crosswalk_id)
                        prev_cw_name = value.ref_crosswalk_id
                        #print(value.ref_crosswalk_id)
                        # print(value.points)
                        for i in range(0,5):
                            min_x = min(min_x,value.points[i][0])
                            min_y = min(min_y,value.points[i][1])
                            max_x = max(max_x,value.points[i][0])
                            max_y = max(max_y,value.points[i][1])
                        


            # intersection_points.append([min_x,min_y,max_x,max_y])
            intersection_points[k] = [min_x,min_y,max_x,max_y]
        
        #print(self.intersection_crosswalk_idx)
        #print(intersection_points)
        
        
        # 횡단보도 구역
        self.cw_points = dict()
        self.line_equation = dict()
        for cw_idx, cw_data in self.cws.items():

            #print(CW_idx)

            self.cw_points[cw_idx] = []
            #self.line_equation[cw_idx] = []

            array = [[0, 0] for row in range(5)]
            array2 = [[0, 0] for row in range(4)]

            cw_min_x,cw_min_y,cw_max_x,cw_max_y = 9999,9999,-9999,-9999
            #print(CW_data.single_crosswalk_list)

            # 횡단보도 한 덩어리 
            for k in cw_data.single_crosswalk_list:
                for i in range(0,5):
                    if cw_min_x > k.points[i][0]:
                        cw_min_x = k.points[i][0]
                        array[0][0] = k.points[i][0]
                        array[0][1] = k.points[i][1]
                    if cw_min_y > k.points[i][1]:
                        cw_min_y = k.points[i][1]
                        array[1][0] = k.points[i][0]
                        array[1][1] = k.points[i][1]
                    if cw_max_x < k.points[i][0]:
                        cw_max_x = k.points[i][0]
                        array[2][0] = k.points[i][0]
                        array[2][1] = k.points[i][1]
                    if cw_max_y < k.points[i][1]:
                        cw_max_y = k.points[i][1]
                        array[3][0] = k.points[i][0]
                        array[3][1] = k.points[i][1]

            # 횡단보도 중심 점, 거리 구하기 ( 원의 중심, 원 반지름 )
            center_point = [0,0,0]
            for i in range(0,4):
                center_point[0] += array[i][0]/4 
                center_point[1] += array[i][1]/4

            # 중심 점으로부터 가장 먼 point까지 거리
            max_dis_cp = -1
            for i in range(0,4):
                dis_cp = sqrt(pow(array[i][0]-center_point[0],2) + pow(array[i][1]-center_point[1],2))
                max_dis_cp = max(max_dis_cp,dis_cp)
            
            center_point[2] = max_dis_cp

            # array[4][0] = array[0][0]
            # array[4][1] = array[0][1]
            
            # # 횡단보도 직선의 방정식 구하기
            # for i in range(0,4):
            #     array2[i][0] = (array[i+1][1] - array[i][1]) / (array[i+1][0] - array[i][0])
            #     array2[i][1] = array[i][1] - array[i][0]*((array[i+1][1] - array[i][1]) / (array[i+1][0] - array[i][0]))
            
            self.cw_points[cw_idx] = center_point
            # self.line_equation[cw_idx] = array2
        #print(self.line_equation)
            
        #print(self.cw_points)

        #global path에서 곡률, target_velocity 설정
        while True:
            if self.is_path == True:
                print(len(self.global_path_msg.poses))
                S = time.time()
                print(S)
                self.velocity_list = self.find_target_velocity()
                E = time.time()
                print(E)
                print("time = ", E-S)
                #self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo('Waiting global path data')
        rate = rospy.Rate(30) # 30hz.
        while not rospy.is_shutdown():

            if self.is_odom == True and self.is_path == True and self.is_status == True:
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
                    
                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(self.local_path_msg)
                
                velocity_msg = Float32()
                # velocity_msg = self.find_target_velocity()
                velocity_msg = self.velocity_list[current_waypoint]

                # n초가 지나면 특정 x, y의 정지선을 무시하는 걸로
                # 정지선을 매번 체크하는 것보다 우회전하기 위해 충분히 속도를 줄였을 때 정지선을 체크하는게 최적화에 도움됨
                # 우회전 조건식 넣어주기
                
                # local path 1 0 = 직진, 1 = 좌회전 2= 우회전
                path_status = -1
                now_intersection_idx = ""
                # 교차로 진입 확인
                is_intersection = False
                crosswalk_check_flag = False

                # for int_idx,int_point in enumerate(intersection_points):
                for int_idx,int_point in intersection_points.items():
                    for lpath_idx in range(len(self.local_path_msg.poses)-1, -1, -1):
                        end_point = self.local_path_msg.poses[lpath_idx].pose.position
                        if int_point[2] > end_point.x > int_point[0] and int_point[3] > end_point.y > int_point[1]:
                            now_intersection_idx = int_idx
                            is_intersection = True
                            break
                    if is_intersection:
                        break
                #print(is_intersection)


                # 신호 감지
                if self.is_traffic_light:
                    #print("신호감지")
                    now_traffic_light_idx = self.traffic_light_data.trafficLightIndex
                    now_traffic_color = self.traffic_light_data.trafficLightStatus
                    # print(f'신호등 인덱스 = {now_traffic_light_idx}, 신호 = {now_traffic_color}')
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
                        people_x = 76
                        people_y = 1210
                        #print(now_intersection_idx)
                        for lpath_idx in range(0,len(self.local_path_msg.poses)):
                            now_local = self.local_path_msg.poses[lpath_idx].pose.position
                            for cw_idx in self.intersection_crosswalk_idx[now_intersection_idx]:
                                #print(cw_idx)
                                # if self.in_crosswalk(cw_idx,now_local.x,now_local.y):
                                #     print(f'{cw_idx} 에 local path')
                                #     crosswalk_check_flag = True
                                #     break

                                # if self.in_crosswalk(cw_idx,people_x,people_y):
                                #     print(f'{cw_idx} 에 사람')
                                if self.in_crosswalk(cw_idx,now_local.x,now_local.y) and self.in_crosswalk(cw_idx,people_x,people_y):
                                    print(f'{cw_idx} 에 사람')
                                    crosswalk_check_flag = True
                                    break
                            
                            if crosswalk_check_flag:
                                break

                        # # 현재 횡단보도 리스트
                        # for sig in list(v.get_signal_list()):
                        #     #print(sig.ref_crosswalk_id)
                        #     for scws_idx,value in self.scws.items():
                        #         # print(
                        #         #     # value.bbox_y,
                        #         #     dir(value)
                        #         #     )
                        #         if value.ref_crosswalk_id == sig.ref_crosswalk_id:
                        #             # print(value.ref_crosswalk_id)
                        #             # print(value.points)
                        #             for i in range(0,5):
                        #                 min_x = min(min_x,value.points[i][0])
                        #                 min_y = min(min_y,value.points[i][1])
                        #                 max_x = max(max_x,value.points[i][0])
                        #                 max_y = max(max_y,value.points[i][1])
                        
                        #print("교차로")
                        #path_status 판단
                        lps_s_point = self.local_path_msg.poses[0].pose.position
                        lps_e_point = self.local_path_msg.poses[99].pose.position

                        degree = self.status_msg.heading - atan2(lps_e_point.y - lps_s_point.y , lps_e_point.x - lps_s_point.x)*(180 / pi)
                        #slope = lps_e_point.x - lps_s_point.x / lps_e_point.y - lps_s_point.y

                        #print(f'곡률 : {self.r}')
                        #print(f'기울기 : {slope}')
                        #print(f'degree = {degree}')

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
                #print(path_status)
                #print(f'target_v = {velocity_msg}')

                # target_velocity = velocity_msg

                # local path경로 위 횡단보도 사람 있을 때
                if crosswalk_check_flag:
                    velocity_msg = 0

                self.velocity_pub.publish(velocity_msg)
                self.is_traffic_light = False

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg

    def traffic_light_callback(self,data):
        self.traffic_light_data = data
        self.is_traffic_light = True

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True
    
    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_data = data 
            
    def find_target_velocity_stoplane(self, stop_lane_pos):
        # 아마 직진거리로 계산할 경우 곡선 형태의 감속 구간에서 언더스티어날듯
        # 노드마다 거리합으로 바꿔줘야됨
        # distance=sqrt(pow(self.x-stop_lane_pos[0],2)+pow(self.y-stop_lane_pos[1],2))
        distance = stop_lane_pos[2]
        # 감속 최대(10m/s^2)
        # print(f'정지선 까지 거리 = {distance}')
        # v^2 = u^2 + 2as
        velocity = Float32()
        velocity = max(0, sqrt(2 * 10 * distance) - 12)
        # print(velocity)
        return velocity

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

        velocity_list =[]

        for i in range(0, len(self.global_path_msg.poses)):

            r = self.find_r(i)

            velocity = Float32()
            velocity = sqrt(r * 9.8 * self.friction)

            if velocity > self.max_velocity:
                velocity = self.max_velocity

            velocity_list.append(velocity)

        return velocity_list

    def find_r(self, now_waypoint):
        
        prev_size = 20
        small_size = 25
        big_size = 100

        big_X_array = []
        big_Y_array = []

        big_r = float('inf')
        small_r = float('inf')
        min_small_r = float('inf')

        # for big_idx in range(0,big_size+1,big_size//2):
        #     x = self.local_path_msg.poses[big_idx].pose.position.x
        #     y = self.local_path_msg.poses[big_idx].pose.position.y
        #     big_X_array.append([x,y,1])
        #     big_Y_array.append([-(x**2)-(y**2)])

        en_b_idx = now_waypoint + big_size+1
        if en_b_idx >= len(self.global_path_msg.poses):
            big_size = len(self.global_path_msg.poses) - now_waypoint - 1

        if big_size > small_size:

            for big_idx in range(now_waypoint,now_waypoint + big_size+1,big_size//2):
                x = self.global_path_msg.poses[big_idx].pose.position.x
                y = self.global_path_msg.poses[big_idx].pose.position.y
                big_X_array.append([x,y,1])
                big_Y_array.append([-(x**2)-(y**2)])

            if(np.linalg.det(big_X_array)):
                    
                X_inverse = np.linalg.inv(big_X_array)

                A_array = X_inverse.dot(big_Y_array)
                a = A_array[0]*-0.5
                b = A_array[1]*-0.5
                c = A_array[2]
                big_r = sqrt(a*a + b*b - c)

        st_s_idx = now_waypoint - prev_size
        en_s_idx = now_waypoint + big_size

        if st_s_idx < 0:
            st_s_idx = 0
        if en_s_idx > len(self.global_path_msg.poses):
            en_s_idx = len(self.global_path_msg.poses)
        
        for path_idx in range(st_s_idx ,en_s_idx - small_size,3):
            
            small_X_array = []
            small_Y_array = []

            for small_idx in range(0,small_size+1,small_size//2):
                x = self.global_path_msg.poses[path_idx+small_idx].pose.position.x
                y = self.global_path_msg.poses[path_idx+small_idx].pose.position.y
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
    
    def in_crosswalk(self,cw_idx, target_x,target_y):

        circle = self.cw_points[cw_idx]

        if (circle[2]+0.5)**2 > pow(target_x - circle[0],2) + pow(target_y - circle[1],2):
            return True
        else:
            return False

        # coef = self.line_equation[cw_idx]
        # print(coef)
        # if target_y < coef[0][0]*target_x + coef[0][1]:
        #     flag = False
        #     print(f'1')

        # if target_y < coef[1][0]*target_x + coef[1][1]:
        #     flag = False
        #     print(f'2')

        # if target_y > coef[2][0]*target_x + coef[2][1]:
        #     flag = False
        #     print(f'3')

        # if target_y > coef[3][0]*target_x + coef[3][1]:
        #     flag = False
        #     print(f'4')

        # if flag:
        #     return True
        # else:
        #     return False

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass