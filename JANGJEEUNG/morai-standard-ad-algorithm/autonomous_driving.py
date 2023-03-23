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

# def EgoStatus_callback(data):
#     rospy.loginfo(data.position.x)
#     rospy.loginfo(data.position.y)
#     rospy.loginfo(data.heading)
#     rospy.sleep(1)
#     raise Exception

# rospy.init_node('Ego_status_listener', anonymous=True)
# callback = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, EgoStatus_callback)
# while not rospy.is_shutdown():
#     try:
#         pass
#     except:
#         break

# 정지선을 파악해야되는 local_path
# local_path = [
#  [126.814949,1308.591064]
# ,[126.987305,1309.064453]
# ,[127.167145,1309.559448]
# ,[127.340874,1310.038818]
# ,[127.505524,1310.493896]
# ,[127.674088,1310.960449]
# ,[127.846054,1311.437134]
# ,[128.021027,1311.922729]
# ,[128.191849,1312.397339]
# ,[128.35936,1312.862915]
# ,[128.528183,1313.332886]
# ,[128.697998,1313.805908]
# ,[128.868393,1314.280884]
# ,[129.03894,1314.756714]
# ,[129.209274,1315.232178]
# ,[129.378998,1315.706177]
# ,[129.547714,1316.177734]
# ,[129.715012,1316.64563]
# ,[129.880936,1317.109619]
# ,[130.055664,1317.598877]
# ,[130.227402,1318.079834]
# ,[130.395676,1318.55127]
# ,[130.559998,1319.011841]
# ,[130.722885,1319.46875]
# ,[130.902649,1319.9729]
# ,[131.074936,1320.456543]
# ,[131.239014,1320.917114]
# ,[131.394119,1321.352783]
# ,[131.579971,1321.875122]
# ,[131.752182,1322.359131]
# ,[131.903732,1322.785278]
# ,[132.093399,1323.319092]
# ,[132.236084,1323.720581]
# ,[132.369217,1324.130615]
# ,[132.515182,1324.630737]
# ,[132.646973,1325.104614]
# ,[132.78447,1325.613037]
# ,[132.907562,1326.077515]
# ,[133.036987,1326.572754]
# ,[133.164215,1327.065308]
# ,[133.28598,1327.54126]
# ,[133.4104,1328.03186]
# ,[133.534943,1328.526367]
# ,[133.655334,1329.007446]
# ,[133.776352,1329.494019]
# ,[133.8974,1329.983521]
# ,[134.017883,1330.473389]
# ,[134.137268,1330.961426]
# ,[134.25499,1331.444824]
# ,[134.373886,1331.935547]
# ,[134.493149,1332.430786]
# ,[134.608505,1332.912354]
# ,[134.723862,1333.396484]
# ,[134.843323,1333.901733]
# ,[134.95491,1334.377197]
# ,[135.070435,1334.87439]
# ,[135.184128,1335.370483]
# ,[135.292618,1335.854248]
# ,[135.394897,1336.332031]
# ,[135.466629,1336.752441]
# ,[135.535187,1337.273071]
# ,[135.5914,1337.758423]
# ,[135.647583,1338.283447]
# ,[135.695923,1338.760864]
# ,[135.745422,1339.270386]
# ,[135.792862,1339.775879]
# ,[135.837524,1340.265869]
# ,[135.882462,1340.77063]
# ,[135.926575,1341.276978]
# ,[135.968887,1341.771851]
# ,[136.01091,1342.271973]
# ,[136.052505,1342.774658]
# ,[136.093491,1343.277344]
# ,[136.133728,1343.777344]
# ,[136.173035,1344.272217]
# ,[136.212845,1344.779663]
# ,[136.251938,1345.284302]
# ,[136.289444,1345.774536]
# ,[136.327835,1346.282104]
# ,[136.36586,1346.791748]
# ,[136.401001,1347.269287]
# ,[136.43927,1347.797729]
# ,[136.473679,1348.283081]
# ,[136.509857,1348.809204]
# ,[136.536163,1349.221069]
# ,[136.55632,1349.680542]
# ,[136.574905,1350.218384]
# ,[136.589401,1350.686035]
# ,[136.604752,1351.215942]
# ,[136.618973,1351.733276]
# ,[136.631546,1352.207764]
# ,[136.644485,1352.709229]
# ,[136.657684,1353.234619]
# ,[136.670105,1353.741211]
# ,[136.681839,1354.226807]
# ,[136.693695,1354.724976]
# ,[136.705627,1355.234131]
# ,[136.717636,1355.752686]
# ,[136.728912,1356.246216]
# ,[136.740189,1356.743408]
# ]

# 정지선을 파악하면 안되는 local_path
local_path = [
[114.598953,1281.826172]
,[114.830986,1282.285156]
,[115.071449,1282.760742]
,[115.30368,1283.219849]
,[115.526947,1283.661011]
,[115.740623,1284.08313]
,[115.95372,1284.503906]
,[116.211342,1285.012207]
,[116.447105,1285.476807]
,[116.659386,1285.894531]
,[116.867149,1286.302612]
,[117.141739,1286.839478]
,[117.307129,1287.158447]
,[117.488457,1287.522095]
,[117.739731,1288.047729]
,[117.928612,1288.448975]
,[118.142052,1288.905884]
,[118.378067,1289.414063]
,[118.576157,1289.842285]
,[118.777924,1290.279785]
,[118.989449,1290.739502]
,[119.209854,1291.219482]
,[119.422104,1291.682617]
,[119.62326,1292.12207]
,[119.829086,1292.57251]
,[120.039101,1293.032471]
,[120.252701,1293.500854]
,[120.464226,1293.965332]
,[120.668701,1294.414551]
,[120.874916,1294.868042]
,[121.082436,1295.324707]
,[121.290787,1295.783691]
,[121.499535,1296.243774]
,[121.708214,1296.704224]
,[121.916405,1297.163696]
,[122.123604,1297.62146]
,[122.329391,1298.076416]
,[122.53334,1298.527466]
,[122.740097,1298.985107]
,[122.952904,1299.456543]
,[123.161987,1299.919922]
,[123.366791,1300.374268]
,[123.566742,1300.818115]
,[123.768532,1301.266479]
,[123.987572,1301.75354]
,[124.197624,1302.221069]
,[124.397774,1302.667114]
,[124.587181,1303.089722]
,[124.8116,1303.591187]
,[125.023949,1304.066895]
,[125.211632,1304.488403]
,[125.430931,1304.983276]
,[125.622696,1305.421265]
,[125.754997,1305.742188]
,[125.959351,1306.274048]
,[126.121178,1306.705322]
,[126.291527,1307.164307]
,[126.480934,1307.678223]
,[126.650749,1308.141479]
,[126.814949,1308.591064]
,[126.987305,1309.064453]
,[127.167145,1309.559448]
,[127.340874,1310.038818]
,[127.505524,1310.493896]
,[127.674088,1310.960449]
,[127.846054,1311.437134]
,[128.021027,1311.922729]
,[128.191849,1312.397339]
,[128.35936,1312.862915]
,[128.528183,1313.332886]
,[128.697998,1313.805908]
,[128.868393,1314.280884]
,[129.03894,1314.756714]
,[129.209274,1315.232178]
,[129.378998,1315.706177]
,[129.547714,1316.177734]
,[129.715012,1316.64563]
,[129.880936,1317.109619]
,[130.055664,1317.598877]
,[130.227402,1318.079834]
,[130.395676,1318.55127]
,[130.559998,1319.011841]
,[130.722885,1319.46875]
,[130.902649,1319.9729]
,[131.074936,1320.456543]
,[131.239014,1320.917114]
,[131.394119,1321.352783]
,[131.579971,1321.875122]
,[131.752182,1322.359131]
,[131.903732,1322.785278]
,[132.093399,1323.319092]
,[132.236084,1323.720581]
,[132.369217,1324.130615]
,[132.515182,1324.630737]
,[132.646973,1325.104614]
,[132.78447,1325.613037]
,[132.907562,1326.077515]
,[133.036987,1326.572754]
,[133.164215,1327.065308]
,[133.28598,1327.54126]
]

np_local_path = np.array(local_path)


#%%

plt.scatter(*np_local_path.T)

ego_size_x = 5
ego_size_y = 1.5
ego_x = 135.18305974645773
ego_y = 1351.6248761140741
ego_angle = 97.4848403930664

rec = patches.Rectangle((ego_x,ego_y), ego_size_x, ego_size_y, angle=ego_angle, color="red")
# plt.gca().add_patch(rec)

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
# plt.show()

points = [
    
# [  113.03095862,  1168.3819978],
# [   81.14295857,  1200.5359978],
# [   80.41295857,  1256.1029978],
# [  147.60595867,  1370.4969978],
# [   77.53695856,  1476.7539978],
# [  149.66395866,  1477.6789978],
# [  128.57695863,  1608.1059978], 
    # [79.599958545819391, 1873.4539978001267], [82.986958549532574, 1873.4809977998957]
    # [
    #     173.06895869743312,
    #     1482.5049978024326,
    #     -0.6400001678466793
    #   ],
    #   [
    #     173.0709586974699,
    #     1484.6519978004508,
    #     -0.6069992218017575
    #   ],
    #   [
    #     173.07395869790344,
    #     1486.7919978015125,
    #     -0.5740001831054684
    #   ],
    #   [
    #     173.07595869811485,
    #     1488.4739977996796,
    #     -0.5479992065429684
    #   ],
    #   [
    #     173.0779586964636,
    #     1490.1439977986738,
    #     -0.522999588012695
    #   ]
      ]

np_points = np.array(points)
xlim = np.average(np_points.T[0])
ylim = np.average(np_points.T[1])
plt.xlim(xlim-50,xlim+50)
plt.ylim(ylim-50,ylim+50)
plt.scatter(np_points.T[0], np_points.T[1])
plt.show()
# %%
# point_lane_boundary_set.sort()
list(filter(lambda x: 136<=x[0]<137 and 1353<=x[1]<1355, point_lane_boundary_set))

# 138.4309586532181,
# 1353.4829978016205,

# 점선
# "lane_type": 525,
# "lane_sub_type": 0,
# "lane_color": "white",
# "lane_shape": [
#     "Broken"
# ],
# "lane_width": 0.15,
# "dash_interval_L1": 0.75,
# "dash_interval_L2": 0.75,
# "double_line_interval": 0.1,

# 실선(정지선)
# "lane_type_def": "",
# "lane_type": 530,
# "lane_sub_type": 0,
# "lane_color": "white",
# "lane_shape": [
#     "Solid"
# ],
# "lane_width": 0.6,
# "dash_interval_L1": 0,
# "dash_interval_L2": 0,
# "double_line_interval": 0.1,