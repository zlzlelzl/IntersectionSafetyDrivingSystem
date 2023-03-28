#!/usr/bin/env python
# -*- coding: utf-8 -*-

from collections import deque
import os
import sys
import rospy
import rospkg
from math import cos, sin, atan2, sqrt, pow, pi
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList
from lib.mgeo.class_defs import *
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
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.object_info_callback)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)

        #TODO: (2) Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)
        self.velocity_pub = rospy.Publisher('/velocity1', Float32, queue_size=1)
        self.stoplane_pub = rospy.Publisher('/stoplane', Path, queue_size=1)
        
        # 초기화
        self.is_odom = False
        self.is_path = False

        #TODO: (3) Local Path 의 Size 결정
        self.local_path_size = 100          # 50 m
        self.max_velocity = 60.0 / 3.6     # 100 km/h
        self.friction = 0.8
        
        self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain = 0.5, distance_gain = 1, time_gap = 0.8, vehicle_length = 2.7)
        
        self.stopped_time = 0
        self.ignore_stoplanes = deque()

        current_path = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(current_path)
        
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        # 정지선 리스트
        lane_boundary_set = mgeo_planner_map.lane_boundary_set
        self.lanes = lane_boundary_set.lanes
        
        # 횡단보도 리스트
        scw_set = mgeo_planner_map.scw_set
        self.scws = scw_set.data
        
        # 횡단보도 리스트
        cw_set = mgeo_planner_map.cw_set
        self.cws = cw_set.data
        
        # # print(self.cws)
        # self.intersection_set = mgeo_planner_map.intersection_controller_set
        # min_x, min_y, max_x, max_y = 9999,9999,-9999,-9999
        # for k,v in self.intersection_set.intersection_controllers.items()[:1]:
        #     print(
        #         # dir(v),

        #         # dir(self.cws[v.get_signal_list()[0].ref_crosswalk_id]),
        #         temp_scw = self.cws[v.get_signal_list()[0].
        #                      ref_crosswalk_id
        #         dir(self.scw[temp_scw].
        #             ref_scw_id["B319BS010044"])
        #         # v.get_signal_list()[0],
        #           )
        # exit()
        
        # # 교차로 리스트
        # scw_set = mgeo_planner_map.scw_set
        # self.scws = scw_set.data
        
        # print(dir(mgeo_planner_map))
        # exit()
        
        self.stoplanes = []
        for lane in self.lanes:
            # 점선은 525 정지선은 530
            if 530 in self.lanes[lane].lane_type:
                self.stoplanes.append(lane)
                        
        # print(dir(scw_set.data["B319BS010062"]),
        #     'bbox_x', 'bbox_y'
        # )
        # exit()
        
        rate = rospy.Rate(30) # 30hz
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

                
                
                # self.current_waypoint = self.get_current_waypoint([self.current_postion.x,self.current_postion.y],self.global_path)
                # self.target_velocity = self.velocity_list[self.current_waypoint]*3.6

                # steering = self.calc_pure_pursuit()
                # if self.is_look_forward_point :
                #     self.ctrl_cmd_msg.steering = steering
                # else : 
                #     rospy.loginfo("no found forward point")
                #     self.ctrl_cmd_msg.steering=0.0

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
                
                # n초가 지나면 특정 x, y의 정지선을 무시하는 걸로
                
                # 정지선을 매번 체크하는 것보다 우회전하기 위해 충분히 속도를 줄였을 때 정지선을 체크하는게 최적화에 도움됨
                # 우회전 조건식 넣어주기
                if 1:
                    # 특정 위치에 정지선이 있는 지 확인
                    stop_lane_pos = self.find_stop_lane_in_local_path()
                    # print("stop_lane_pos : ", stop_lane_pos)
                    
                    # 값이 있으면 속도 덮어씌우기
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
                
                # # global_obj,local_obj
                # result = self.calc_vaild_obj([x,y,self.vehicle_yaw],self.object_data)
                
                # global_npc_info = result[0] 
                # local_npc_info = result[1] 
                # global_ped_info = result[2] 
                # local_ped_info = result[3] 
                # global_obs_info = result[4] 
                # local_obs_info = result[5]
                
                # self.adaptive_cruise_control.check_object(self.global_path_msg ,global_npc_info, local_npc_info
                #                                                     ,global_ped_info, local_ped_info
                #                                                     ,global_obs_info, local_obs_info)
                # print(velocity_msg)
                # velocity_msg = self.adaptive_cruise_control.get_target_velocity(local_npc_info, local_ped_info, local_obs_info,self.status_msg.velocity.x, velocity_msg / 3.6)
                # print(velocity_msg)
                
                self.velocity_pub.publish(velocity_msg)

            rate.sleep()

    def calc_vaild_obj(self,status_msg,object_data):
        
        self.all_object = object_data        
        ego_pose_x = status_msg[0]
        ego_pose_y = status_msg[1]
        ego_heading = status_msg[2]
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian        
        if num_of_object > 0:

            #translation
            tmp_theta=ego_heading
            tmp_translation=[ego_pose_x, ego_pose_y]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0             ,               0,                  1]])
            tmp_det_t=np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0,0,1]])

            #npc vehicle ranslation        
            for npc_list in self.all_object.npc_list:
                global_result=np.array([[npc_list.position.x],[npc_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :        
                    global_npc_info.append([npc_list.type,npc_list.position.x,npc_list.position.y,npc_list.velocity.x])
                    local_npc_info.append([npc_list.type,local_result[0][0],local_result[1][0],npc_list.velocity.x])

            #ped translation
            for ped_list in self.all_object.pedestrian_list:
                global_result=np.array([[ped_list.position.x],[ped_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_ped_info.append([ped_list.type,ped_list.position.x,ped_list.position.y,ped_list.velocity.x])
                    local_ped_info.append([ped_list.type,local_result[0][0],local_result[1][0],ped_list.velocity.x])

            #obs translation
            for obs_list in self.all_object.obstacle_list:
                global_result=np.array([[obs_list.position.x],[obs_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_obs_info.append([obs_list.type,obs_list.position.x,obs_list.position.y,obs_list.velocity.x])
                    local_obs_info.append([obs_list.type,local_result[0][0],local_result[1][0],obs_list.velocity.x])
                
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info, local_obs_info

    def object_info_callback(self,data): ## Object information Subscriber
        self.is_object_info = True
        self.object_data = data 

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg    

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 현재 위치를 저장
        
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg
            
    def find_target_velocity_stoplane(self, stop_lane_pos):
        # 아마 직진거리로 계산할 경우 곡선 형태의 감속 구간에서 언더스티어날듯
        # 노드마다 거리합으로 바꿔줘야됨
        # distance=sqrt(pow(self.x-stop_lane_pos[0],2)+pow(self.y-stop_lane_pos[1],2))
        distance = stop_lane_pos[2]
        # 감속 최대(10m/s^2)
        
        # v^2 = u^2 + 2as
        velocity = Float32()
        velocity = max(0, sqrt(2 * 10 * distance) - 15)
        
        # print(velocity)
        return velocity
    
    def find_stop_lane_in_local_path(self):
        curve_distance=0
        prev_x, prev_y = self.x, self.y
        for p in self.local_path_msg.poses:
            curve_distance+=sqrt(pow(prev_x-p.pose.position.x,2)+pow(prev_y-p.pose.position.y,2))
            prev_x, prev_y = p.pose.position.x, p.pose.position.y
            # print(self.ignore_stoplanes)
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

class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.npc_vehicle=[False,0]
        self.object=[False,0]
        self.Person=[False,0]
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length

        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0

    def check_object(self,ref_path, global_npc_info, local_npc_info, 
                                    global_ped_info, local_ped_info, 
                                    global_obs_info, local_obs_info):
        #TODO: (8) 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)
        min_rel_distance=float('inf')
        if len(global_ped_info) > 0 :        
            for i in range(len(global_ped_info)):
                for path in ref_path.poses :      
                    if global_ped_info[i][0] == 0 : # type=0 [pedestrian]                    
                        dis=sqrt(pow(path.pose.position.x-global_ped_info[i][1],2)+pow(path.pose.position.y-global_ped_info[i][2],2))
                        if dis<2.35:                            
                            rel_distance= sqrt(pow(local_ped_info[i][1],2)+pow(local_ped_info[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.Person=[True,i]

        if len(global_npc_info) > 0 :            
            for i in range(len(global_npc_info)):
                for path in ref_path.poses :      
                    if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
                        dis=sqrt(pow(path.pose.position.x-global_npc_info[i][1],2)+pow(path.pose.position.y-global_npc_info[i][2],2))
                        if dis<2.35:
                            rel_distance= sqrt(pow(local_npc_info[i][1],2)+pow(local_npc_info[i][2],2))                            
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.npc_vehicle=[True,i]

        if len(global_obs_info) > 0 :            
            for i in range(len(global_obs_info)):
                for path in ref_path.poses :      
                    if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
                        dis=sqrt(pow(path.pose.position.x-global_obs_info[i][1],2)+pow(path.pose.position.y-global_obs_info[i][2],2))
                        if dis<2.35:
                            rel_distance= sqrt(pow(local_obs_info[i][1],2)+pow(local_obs_info[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                # self.object=[True,i] 

    def get_target_velocity(self, local_npc_info, local_ped_info, local_obs_info, ego_vel, target_vel): 
        #TODO: (9) 장애물과의 속도와 거리 차이를 이용하여 ACC 를 진행 목표 속도를 설정
        out_vel = target_vel
        default_space = 20
        time_gap = self.time_gap
        v_gain = self.velocity_gain
        x_errgain = self.distance_gain

        if self.npc_vehicle[0] and len(local_npc_info) != 0: #ACC ON_vehicle   
            print("ACC ON NPC_Vehicle")         
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
            
            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
            vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            out_vel = ego_vel + acceleration      

        if self.Person[0] and len(local_ped_info) != 0: #ACC ON_Pedestrian
            print("ACC ON Pedestrian")
            Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
            vel_rel = (Pedestrian[2] - ego_vel)              
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration
   
        if self.object[0] and len(local_obs_info) != 0: #ACC ON_obstacle     
            print("ACC ON Obstacle")                    
            Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))            
            vel_rel = (Obstacle[2] - ego_vel)
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration           

        return out_vel * 3.6


if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass