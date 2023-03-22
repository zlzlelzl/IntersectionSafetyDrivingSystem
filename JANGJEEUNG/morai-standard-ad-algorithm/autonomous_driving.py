#%%
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from perception.forward_object_detector import ForwardObjectDetector
from localization.path_manager import PathManager
from planning.adaptive_cruise_control import AdaptiveCruiseControl
from control.pure_pursuit import PurePursuit
from control.pid import Pid
from control.control_input import ControlInput
from config.config import Config

import numpy as np
import matplotlib.pyplot as plt
import json

import matplotlib.patches as patches
import matplotlib as mpl

import rospy
from morai_msgs.msg import EgoVehicleStatus


from mgeo.calc_mgeo_path import mgeo_dijkstra_path


class AutonomousDriving:
    def __init__(self):
        
        config = Config()
        config.update_config("config/config.json")

        if config["map"]["use_mgeo_path"]:
            mgeo_path = mgeo_dijkstra_path(config["map"]["name"])
            self.path = mgeo_path.calc_dijkstra_path(config["map"]["mgeo"]["start_node"], config["map"]["mgeo"]["end_node"])
            self.path_manager = PathManager(
                self.path, config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )
        else:
            self.path = config["map"]["path"]
            self.path_manager = PathManager(
                self.path , config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )
        self.path_manager.set_velocity_profile(**config['planning']['velocity_profile'])

        # self.forward_object_detector = ForwardObjectDetector(config["map"]["traffic_light_list"])

        self.adaptive_cruise_control = AdaptiveCruiseControl(
            vehicle_length=config['common']['vehicle_length'], **config['planning']['adaptive_cruise_control']
        )
        self.pid = Pid(sampling_time=1/float(config['common']['sampling_rate']), **config['control']['pid'])
        self.pure_pursuit = PurePursuit(
            wheelbase=config['common']['wheelbase'], **config['control']['pure_pursuit']
        )

    def execute(self, vehicle_state, dynamic_object_list, current_traffic_light):
        # 현재 위치 기반으로 local path과 planned velocity 추출
        local_path, planned_velocity = self.path_manager.get_local_path(vehicle_state)

        # 전방 장애물 인지
        self.forward_object_detector._dynamic_object_list = dynamic_object_list
        object_info_dic_list = self.forward_object_detector.detect_object(vehicle_state)

        # adaptive cruise control를 활용한 속도 계획
        self.adaptive_cruise_control.check_object(local_path, object_info_dic_list, current_traffic_light)
        target_velocity = self.adaptive_cruise_control.get_target_velocity(vehicle_state.velocity, planned_velocity)
        # 속도 제어를 위한 PID control
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)
        # target velocity가 0이고, 일정 속도 이하일 경우 full brake를 하여 차량을 멈추도록 함.
        if round(target_velocity) == 0 and vehicle_state.velocity < 2:
            acc_cmd = -1.
        # 경로 추종을 위한 pure pursuit control
        self.pure_pursuit.path = local_path
        self.pure_pursuit.vehicle_state = vehicle_state
        steering_cmd = self.pure_pursuit.calculate_steering_angle()

        return ControlInput(acc_cmd, steering_cmd), local_path


# if __name__  =="__main__":
if 1:
    ad = AutonomousDriving()
    # print(dir(ad))
    li = [
        # ad.adaptive_cruise_control,
        #   ad.forward_object_detector,
          ad.path,
        #   ad.path_manager,
        #   ad.pid,
        #   ad.pure_pursuit
        ]
    
    class Point:
        def __init__(self, arr) -> None:
            self.x = arr[0]
            self.y = arr[1]
        
    a = []    
    for l in li:
        a.append(l)
        
    path = []
    for i in range(len(a[0])):
        path.append([a[0][i].x, a[0][i].y])
        
    np_path = np.array(path)
    
    # plt.show()
    
    
# 모든 노드와 링크를 캔버스에 나타내기
link_set = None
node_set = None
lane_boundary_set = None
lane_node_set = None
singlecrosswalk_set = None
surface_marking_set = None

with open("json1/link_set.json", "r",encoding="utf-8" ) as f:
    link_set = json.load(f)

with open("json1/node_set.json", "r",encoding="utf-8" ) as f:
    node_set = json.load(f)
    
with open("json1/lane_boundary_set.json", "r",encoding="utf-8" ) as f:
    lane_boundary_set = json.load(f)
    
with open("json1/lane_node_set.json", "r",encoding="utf-8" ) as f:
    lane_node_set = json.load(f)
    
with open("json1/singlecrosswalk_set.json", "r",encoding="utf-8" ) as f:
    singlecrosswalk_set = json.load(f)

with open("json1/surface_marking_set.json", "r",encoding="utf-8" ) as f:
    surface_marking_set = json.load(f)
    
with open("json1/traffic_light_set.json", "r",encoding="utf-8" ) as f:
    traffic_light_set = json.load(f)
    
with open("json1/traffic_sign_set.json", "r",encoding="utf-8" ) as f:
    traffic_sign_set = json.load(f)



#%%

# link_set의 포인트는 작은 흰점
# node_set의 포인트는 큰 노란점

# point_singlecrosswalk_set = []
# point_surface_marking_set = []

## 링크 셋

point_singlecrosswalk_set = []

for i in range(len(singlecrosswalk_set)):
    for j in range(len(singlecrosswalk_set[i])):
        for k in range(len(singlecrosswalk_set[i]["points"])):
            point_singlecrosswalk_set.append(singlecrosswalk_set[i]["points"][k][:2])

point_surface_marking_set = []

for i in range(len(surface_marking_set)):
    for j in range(len(surface_marking_set[i])):
        for k in range(len(surface_marking_set[i]["points"])):
            point_surface_marking_set.append(surface_marking_set[i]["points"][k][:2])

point_lane_boundary_set = []

for i in range(len(lane_boundary_set)):
    for j in range(len(lane_boundary_set[i])):
        for k in range(len(lane_boundary_set[i]["points"])):
            point_lane_boundary_set.append(lane_boundary_set[i]["points"][k][:2])

point_link_set = []

for i in range(len(link_set)):
    for j in range(len(link_set[i])):
        for k in range(len(link_set[i]["points"])):
            point_link_set.append(link_set[i]["points"][k][:2])


## 노드 셋

point_traffic_light_set = []

for i in range(len(traffic_light_set)):
    for j in range(len(traffic_light_set[i])):
            point_traffic_light_set.append(traffic_light_set[i]["point"][:2])
            
point_traffic_sign_set = []

for i in range(len(traffic_sign_set)):
    for j in range(len(traffic_sign_set[i])):
            point_traffic_sign_set.append(traffic_sign_set[i]["point"][:2])


point_lane_node_set = []

for i in range(len(lane_node_set)):
    for j in range(len(lane_node_set[i])):
            point_lane_node_set.append(lane_node_set[i]["point"][:2])

point_node_set = []

for i in range(len(node_set)):
    for j in range(len(node_set[i])):
            point_node_set.append(node_set[i]["point"][:2])

np_link = np.array(point_link_set)
np_node = np.array(point_node_set)
np_lane_boundary_set = np.array(point_lane_boundary_set)
np_lane_node_set = np.array(point_lane_node_set)
np_singlecrosswalk_set = np.array(point_singlecrosswalk_set)
np_surface_marking_set = np.array(point_surface_marking_set)

np_traffic_light_set = np.array(point_traffic_light_set)
np_traffic_sign_set = np.array(point_traffic_sign_set)

# plt.scatter(*np_path.T, c ="white", label="path")


# %%

def EgoStatus_callback(data):
    rospy.loginfo(data.position.x)
    rospy.loginfo(data.position.y)
    rospy.loginfo(data.heading)
    rospy.sleep(1)
    raise Exception

rospy.init_node('Ego_status_listener', anonymous=True)
callback = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, EgoStatus_callback)
while not rospy.is_shutdown():
    try:
        pass
    except:
        break
        


#%%
ego_size_x = 5
ego_size_y = 1.5
ego_x = 140.9760284423828
ego_y = 1405.075439453125
ego_angle = 97.4848403930664

rec = patches.Rectangle((ego_x,ego_y), ego_size_x, ego_size_y, angle=ego_angle, color="red")
plt.gca().add_patch(rec)

# 차체 중심
plt.xlim([ego_x - 50 , ego_x + 50])
plt.ylim([ego_y - 50 , ego_y + 50])


# # 교차로 시작 북쪽
# plt.xlim([120, 160])
# plt.ylim([1500, 1525])

# # 교차로 전체
# plt.xlim([50, 200])
# plt.ylim([1300, 1700])

# # 교차로 중심
# plt.xlim([80, 180])
# plt.ylim([1450, 1550])

# 차선 바운더리
plt.scatter(*np_lane_boundary_set.T, c="blue", s = 1, label="boundary")
# plt.scatter(*np_lane_node_set.T, c="red", s = 5, label="lane_node")

# # 차선 노드, 링크
# plt.scatter(*np_link.T, c="green", s = 1, label="link" )
# plt.scatter(*np_node.T, c="blue", s = 5, label="node")

# # 횡단 보도
# plt.scatter(*np_singlecrosswalk_set.T, c="red",s = 5, label="crosswalk")
# # 도로 마커
# plt.scatter(*np_surface_marking_set.T, c="black", s = 1, label="marker")

# # 신호(차로)
# plt.scatter(*np_traffic_light_set.T, c="red", s = 2, label="crosswalk")
# # 신호(횡단보도)
# plt.scatter(*np_traffic_sign_set.T, c="black", s = 1, label="marker")




plt.legend(loc='best')
plt.show()


# %%
