#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
import sys
from std_msgs.msg import String,Float32
from nav_msgs.msg import Path,Odometry
from morai_msgs.msg import EgoVehicleStatus,TrafficSignInfo,ObjectStatusList
from lib.mgeo.class_defs import *
from collections import deque
import heapq
from math import cos, sin, atan2, sqrt, pow, pi
from tf.transformations import euler_from_quaternion
# my_name_listener 는 Custom Msgs 를 이용한 Topic Subscriber(메세지 수신) 예제입니다.
# /my_name 라는 메세지를 Subscribe 합니다.

# 노드 실행 순서 
# 1. ROS 노드 이름 선언
# 2. Subscriber 생성
# 3. Callback 함수 생성 및 데이터 출력

#TODO: (3) Callback 함수 생성 및 데이터 출력

class intersection_decision :
    def __init__(self):
        rospy.init_node('intersection_decision', anonymous=True)

        rospy.Subscriber( 'traffic_sign_info' , TrafficSignInfo , self.traffic_light_callback)
        rospy.Subscriber('/local_path',Path, self.local_path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)

        self.velocity_pub = rospy.Publisher('/velocity3', Float32, queue_size=1)

        # MGeo data 불러오기
        current_path = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(current_path)
        
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        # 실선, 점선 data
        lane_boundary_set = mgeo_planner_map.lane_boundary_set
        self.lanes = lane_boundary_set.lanes

        # 횡단보도 data
        cw_set = mgeo_planner_map.cw_set
        self.cws = cw_set.data
        scw_set = mgeo_planner_map.scw_set
        self.scws = scw_set.data

        # 교차로 data
        self.intersection_set = mgeo_planner_map.intersection_controller_set

        # mgeo에서 신호,교차로 정보 받아오기
        self.traffic_set = mgeo_planner_map.light_set

        # 횡단보도 data
        cw_set = mgeo_planner_map.cw_set
        self.cws = cw_set.data
        scw_set = mgeo_planner_map.scw_set
        self.scws = scw_set.data

        # 정지선 변수
        self.stopped_time = 0
        self.ignore_stoplanes = deque()
        # 신호 제어용 변수
        self.traffic_stopped_time = 0
        #정지선 list
        self.stoplanes = self.stoplane_setting(self.lanes)


        # 교차로 boundary, 교차로 횡단보도 idx 저장 
        self.intersection_points, self.intersection_crosswalk_idx = self.intersection_boundary_setting(self.intersection_set, self.scws)
        self.intersection_points['IntTL1'] = [122,1595,146,1625]
        self.intersection_points['IntTL5'] = [116,1353,153,1384]
        print(self.intersection_points)
        # 횡단보도 boundary 저장 
        self.cw_points = self.crosswalk_boundary_setting(self.cws)

        self.is_traffic_light = False
        self.is_local_path = False
        self.is_status = False
        self.is_object_info = False

        self.max_velocity = 60 / 3.6
        # 현재 신호
        self.traffic_light_sign = 0
        
        q = []
        # t_l_count[green,red]
        traffic_light_queue = []
        traffic_light_count = [0,0]
        prev_stop_lane = [0,0]

        rate = rospy.Rate(30) # 30hz.
        while not rospy.is_shutdown():

            if self.is_local_path and self.is_status and self.is_traffic_light:

                #velocity 초기화
                velocity_msg = self.max_velocity

                # 교차로 진입 초기화 
                is_intersection = False
                path_status = -1


                # 횡단보도 진입 초기화 
                crosswalk_check_flag = False

                # for lpath_idx in range(len(self.local_path_msg.poses)-1, -1, -1):
                #     for int_idx,int_point in self.intersection_points.items():
                #         end_point = self.local_path_msg.poses[lpath_idx].pose.position
                #         if int_point[2] > end_point.x > int_point[0] and int_point[3] > end_point.y > int_point[1]:
                #             now_intersection_idx = int_idx
                #             is_intersection = True
                #             # print('{0}에 {1} 진입'.format(lpath_idx, int_idx))
                #             break
                #     if is_intersection:
                #         break

                now_intersection_idx = []
                prev_int_idx = ""
                for lpath_idx in range(len(self.local_path_msg.poses)-1, -1, -1):
                # for lpath_idx in range(0,len(self.local_path_msg.poses)):
                        for int_idx,int_point in self.intersection_points.items():
                            end_point = self.local_path_msg.poses[lpath_idx].pose.position
                            if int_point[2] > end_point.x > int_point[0] and int_point[3] > end_point.y > int_point[1]:
                                # now_intersection_idx = int_idx
                                if int_idx != prev_int_idx:
                                    now_intersection_idx.append(int_idx)
                                prev_int_idx = int_idx
                                is_intersection = True
                                # print('{0}에 {1} 진입'.format(lpath_idx, int_idx))
                                break
                        if is_intersection:
                            break
                # 정지선 인지, 거리 계산
                stop_lane_pos = self.find_stop_lane_in_local_path()
                if len(stop_lane_pos) != 0:
                        
                        stop_lane_dis = stop_lane_pos[2]
                        now_stop_lane = [stop_lane_pos[0],stop_lane_pos[1]]
                        # print(f'prev = {prev_stop_lane}, now = {now_stop_lane}')
                        if prev_stop_lane != now_stop_lane:
                            print("정지선 넘어감")
                            traffic_light_queue = deque()
                            traffic_light_count = [0,0]

                        prev_stop_lane = now_stop_lane

                if is_intersection:

                    if len(self.traffic_light_data.traffic_light) > 0:

                        now_traffic_light = self.traffic_light_data.traffic_light[0]

                        traffic_light_status = now_traffic_light.traffic_light_status

                        traffic_light_accuracy = now_traffic_light.detect_precision
                        
                        if traffic_light_accuracy >0.4:

                            traffic_light_queue.append(traffic_light_status)
                            print(traffic_light_queue)
                            if traffic_light_status == 0 or traffic_light_status == 5:
                                traffic_light_count[0] += 1
                            elif traffic_light_status == 1:
                                traffic_light_count[1] += 2

                            if len(traffic_light_queue) >= 10:
                                temp = traffic_light_queue.popleft()

                                if temp == 0:
                                    traffic_light_count[0] -= 1
                                elif temp == 1:
                                    traffic_light_count[1] -= 2
                        
                        if traffic_light_count[0] > traffic_light_count[1]:
                            now_color = 0
                        else :
                            now_color = 1 
                        print(traffic_light_count)
                        
                        if traffic_light_status == 3:
                            now_color = 3


                        traffic_light_status = now_color
                        
                        # if now_traffic_light.traffic_light_type not in [13, 14]:
                        print(f'신호 = {traffic_light_status}, type = {now_traffic_light.traffic_light_type}')

                        #정지 (빨 + 빨+좌)
                        if traffic_light_status == 1:
                            
                            self.traffic_light_sign = 1

                            if len(stop_lane_pos) != 0:
                                velocity_msg = self.find_target_velocity_stoplane(stop_lane_dis)
                                # if velocity_msg > 7 and stop_lane_dis < 20:
                                #     velocity_msg /= 5
                                # # velocity_msg = 0

                        #직진 (직, 직+좌)
                        elif traffic_light_status == 0 or traffic_light_status == 5 :
                            self.traffic_light_sign = 0

                    # people_list = [[80,1192]] # [100,1236] [76,1210] [83,1234] [97,1251]
                    # stop_crosswalk_list = []
                    # stop_crosswalk_lpath_idx = -1
                    # #print(now_intersection_idx)
                    # for lpath_idx in range(0,len(self.local_path_msg.poses)):
                    #     now_local = self.local_path_msg.poses[lpath_idx].pose.position
                    #     for cw_idx in self.intersection_crosswalk_idx[now_intersection_idx]:
                    #         #print(cw_idx)
                    #         # if self.in_crosswalk(cw_idx,now_local.x,now_local.y):
                    #         #     print(f'{cw_idx} 에 local path')
                    #         #     crosswalk_check_flag = True
                    #         #     break

                    #         # if self.in_crosswalk(cw_idx,people_x,people_y):
                    #         #     print(f'{cw_idx} 에 사람')
                    #         for idx,people in enumerate(people_list):
                                
                    #             if self.in_crosswalk(cw_idx,now_local.x,now_local.y) and self.in_crosswalk(cw_idx,people[0],people[1]):
                    #                 #print(f'{cw_idx}, local{lpath_idx}번 에 ({people_list[idx][0]},{people_list[idx][1]}) 사람')
                    #                 crosswalk_check_flag = True
                    #                 stop_crosswalk_lpath_idx = lpath_idx
                    #                 if cw_idx not in stop_crosswalk_list:
                    #                     stop_crosswalk_list.append(cw_idx)
                                    
                            
                    #         if crosswalk_check_flag:
                    #             break
                        
                    #     if crosswalk_check_flag:
                    #         break

                    # 사람 확인
                    people_list = []
                    # people_list = [[100,1236]] # [76,1210] [83,1234] [97,1251]
                    for ped_list in self.object_data.pedestrian_list:
                        people_list.append([ped_list.position.x,ped_list.position.y])
                    # people_list = [[53,1206]]
                    people_list = [[80,1192]]
                    # 사람있는 횡단보도
                    stop_crosswalk_list = []
                    stop_crosswalk_lpath_idx = -1

                    if len(people_list) > 0:
                        print("사람 감지 ")
                        print(people_list)
                        for lpath_idx in range(0,len(self.local_path_msg.poses)):
                            now_local = self.local_path_msg.poses[lpath_idx].pose.position
                            for int_idx in now_intersection_idx:
                                for cw_idx in self.intersection_crosswalk_idx[int_idx]:

                                    for idx,person in enumerate(people_list):
                                        print(person)
                                        if self.in_crosswalk(cw_idx,now_local.x,now_local.y) and self.in_crosswalk(cw_idx,person[0],person[1]):
                                            crosswalk_check_flag = True
                                            print("횡단보도 건너s느 사람 감지 ")
                                            stop_crosswalk_lpath_idx = lpath_idx
                                            if cw_idx not in stop_crosswalk_list:
                                                stop_crosswalk_list.append(cw_idx)
                                        # if self.in_crosswalk(cw_idx,now_local.x,now_local.y):
                                        #     print("local in")
                                        # if self.in_crosswalk(cw_idx,person[0],person[1]):
                                        #     print("person in")
                                        
                                
                                if crosswalk_check_flag:
                                    break
                            
                            if crosswalk_check_flag:
                                break

                        lps_s_point = self.local_path_msg.poses[0].pose.position
                        lps_e_point = self.local_path_msg.poses[99].pose.position

                        # print("yaw = {0}, heading = {1}".format(self.vehicle_yaw,self.status_msg.heading))
                        degree = self.vehicle_yaw - atan2(lps_e_point.y - lps_s_point.y , lps_e_point.x - lps_s_point.x)
                        degree = degree * (180 / pi)

                        if -30 < degree < 30:
                            path_status = 0
                        elif 30< degree < 60:
                            path_status = 2
                        elif -30> degree > -60:
                            path_status = 1

                        if path_status == 1:  # 좌회전

                            #정지 (빨간불)
                            if traffic_light_status == 1:

                                if len(stop_lane_pos) != 0:
                                    dis_traffic_light = stop_lane_pos[2]
                                    velocity_msg = self.find_target_velocity_stoplane(stop_lane_dis)


                            #직진 (직, 직+좌)
                            else:
                                pass

                        elif path_status == 2:  # 우회전

                            if len(stop_lane_pos) != 0:
                                velocity_msg = self.find_target_velocity_stoplane(stop_lane_dis)
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
                    else:
                        pass
                        if self.traffic_light_sign == 1:
                            velocity_msg = self.find_target_velocity_stoplane(stop_lane_dis)
                            # velocity_msg = 0
                        elif self.traffic_light_sign == 0 or traffic_light_status == 5:
                            velocity_msg = self.max_velocity
                        

                else:
                    pass

                
                # local path경로 위 횡단보도 사람 있을 때
                velocity_msg2 = self.max_velocity

                if crosswalk_check_flag:

                    curve_distance=0
                    prev_x, prev_y = self.x, self.y

                    for i in range(0, stop_crosswalk_lpath_idx+1):
                        p = self.local_path_msg.poses[i]
                        curve_distance+=sqrt(pow(prev_x-p.pose.position.x,2)+pow(prev_y-p.pose.position.y,2))
                        prev_x, prev_y = p.pose.position.x, p.pose.position.y

                    # print(curve_distance)
                    print("사람 속도 줄이기 ")
                    velocity_msg2 = self.find_target_velocity_stoplane(curve_distance)
                    # if curve_distance < 10 and velocity_msg2 > 5:
                    #     velocity_msg2 = 0


                velocity_msg = min(velocity_msg,velocity_msg2)
                # # 멈춰있을 때 무조건 시간 늘이기
                # if velocity_msg <= 2:
                #     self.traffic_stopped_time += 1
                #     # print(self.ignore_stoplanes, self.stopped_time)
                # else:
                #     self.traffic_stopped_time = 0
                # print(velocity_msg)
                # print(f'속도 = {velocity_msg}')
                self.velocity_pub.publish(velocity_msg)

            rate.sleep()

    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_data = data 

    def traffic_light_callback(self,data):
        self.traffic_light_data = data
        self.is_traffic_light = True

        if len(self.traffic_light_data.traffic_light) > 0:


            for i in range(len(data.traffic_light)):
                d = data.traffic_light[i]
                sign_type = d.traffic_light_type
                sign_status = d.traffic_light_status
                # print('#'+str(i), sign_type, sign_status)


    def local_path_callback(self,msg):
        self.local_path_msg = msg
        self.is_local_path = True

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)

    def stoplane_setting(self, lanes):

            stoplanes = []

            for lane in lanes:
                # 점선은 525 정지선은 530
                if 530 in lanes[lane].lane_type:
                    stoplanes.append(lane)
                    
            return stoplanes

    def intersection_boundary_setting(self, intersection_set, scws):
            intersection_points = dict()
            intersection_crosswalk_idx = dict()

            for int_idx,int_data in intersection_set.intersection_controllers.items():

                min_x, min_y, max_x, max_y = 9999,9999,-9999,-9999
                intersection_crosswalk_idx[int_idx] = []
                prev_cw_name = ""

                for sig in list(int_data.get_signal_list()):

                    for scws_idx,scws_data in scws.items():

                        if scws_data.ref_crosswalk_id == sig.ref_crosswalk_id:

                            if prev_cw_name != scws_data.ref_crosswalk_id:
                                intersection_crosswalk_idx[int_idx].append(scws_data.ref_crosswalk_id)

                            prev_cw_name = scws_data.ref_crosswalk_id

                            for i in range(0,5):
                                min_x = min(min_x,scws_data.points[i][0])
                                min_y = min(min_y,scws_data.points[i][1])
                                max_x = max(max_x,scws_data.points[i][0])
                                max_y = max(max_y,scws_data.points[i][1])

                intersection_points[int_idx] = [min_x,min_y,max_x,max_y]
            
            return intersection_points, intersection_crosswalk_idx
    
    def crosswalk_boundary_setting(self,cws):
            
            cw_points = dict()

            for cw_idx, cw_data in cws.items():

                cw_points[cw_idx] = []

                array = [[0, 0] for row in range(4)]
                
                min_x,min_y,max_x,max_y = 9999,9999,-9999,-9999

                # 횡단보도 한 덩어리 
                for sw in cw_data.single_crosswalk_list:
                    for i in range(0,5):
                        if min_x > sw.points[i][0]:
                            min_x = sw.points[i][0]
                            array[0][0] = sw.points[i][0]
                            array[0][1] = sw.points[i][1]
                        if min_y > sw.points[i][1]:
                            min_y = sw.points[i][1]
                            array[1][0] = sw.points[i][0]
                            array[1][1] = sw.points[i][1]
                        if max_x < sw.points[i][0]:
                            max_x = sw.points[i][0]
                            array[2][0] = sw.points[i][0]
                            array[2][1] = sw.points[i][1]
                        if max_y < sw.points[i][1]:
                            max_y = sw.points[i][1]
                            array[3][0] = sw.points[i][0]
                            array[3][1] = sw.points[i][1]

                # 횡단보도 중심 점, 거리 구하기 ( 원의 중심(x,y), 원 반지름 )
                center_point = [0,0,0]
                for i in range(0,4):
                    center_point[0] += array[i][0]/4 
                    center_point[1] += array[i][1]/4

                # 중심 점으로부터 가장 먼 point까지 거리
                max_dis_cp = -1
                for i in range(0,4):
                    dis_cp = sqrt(pow(array[i][0]-center_point[0],2) + pow(array[i][1]-center_point[1],2))
                    max_dis_cp = max(max_dis_cp,dis_cp)
                
                #원 반지름
                center_point[2] = max_dis_cp
                
                cw_points[cw_idx] = center_point

            return cw_points
    
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
    

    def find_target_velocity_stoplane(self, distance):

        velocity = Float32()
        velocity = max(0, sqrt(2 * 9 * distance) - 11 ) 

        return velocity
    
    def in_crosswalk(self,cw_idx, target_x,target_y):

        circle = self.cw_points[cw_idx]

        if pow((circle[2]+0.5),2) > pow(target_x - circle[0],2) + pow(target_y - circle[1],2):
            return True
        else:
            return False

if __name__ == '__main__':
    try:
        intersection=intersection_decision()
    except rospy.ROSInterruptException:
        pass
