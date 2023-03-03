#!/usr/bin/env python3
from std_msgs.msg import String
from morai_msgs.srv import MoraiMapSpecSrv
from morai_msgs.msg import EgoVehicleStatus
from autonomous_driving import AutonomousDriving
from vehicle_state import VehicleState
import rospy

import numpy as np
import matplotlib.pyplot as pyplot



def EgoStatus_callback(data):
    rospy.loginfo('------------------Ego Vehicle Status------------------')
    
    _input = ad.execute(VehicleState(data.velocity.x, data.velocity.y,0,0), [], [])
    # np_input = np.array(_input)
    
    ctrl, point = _input
    np_point = np.array(point)
    
    # print(point)
    # pyplot.scatter(*np_point.T)
    # pyplot.show()
    
    # rospy.loginfo('position     : x = {0} , y = {1}, z = {2}'.format(data.))
    '''
    rospy.loginfo('velocity     : x = {0} , y = {1}, z = {2} m/s^2'.format( Ego 차량의 x y z 좌표 속도 데이터를 입력합니다. ))
    rospy.loginfo('acceleration : x = {0} , y = {1}, z = {2} m/s'.format( Ego 차량의 x y z 좌표 가속도 데이터를 입력합니다. ))
    rospy.loginfo('heading      : {} deg'.format( Ego 차량의 heading 값을 입력합니다.))
    '''
    
def listener():
    rospy.init_node('ego_topic', anonymous=True)
    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, EgoStatus_callback)
    # rospy.init_node('listener', anonymous=True)
    # rospy.wait_for_service('/Service_MoraiMapSpec')
    # event_cmd_srv = rospy.ServiceProxy('Service_MoraiMapSpec', MoraiMapSpecSrv)
    # print(event_cmd_srv())
    # rospy.Subscriber("/map", OccupancyGrid, callback)
    # rospy.wait_for_service("Service_MoraiMapSpec")
    # try:
    #     print(Service_MoraiMapSpec)
    # except:
    #     pass
    
    # rospy.wait_for_service("/Service_MoraiMapSpec")
    # service_client = rospy.ServiceProxy("/Service_MoraiMapSpec", "bool", callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        hello_str = "hello SSAFY %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

# 맵은 갖다쓰기
# vehicle state 만들기
ad = AutonomousDriving()
rospy.init_node('ego_topic', anonymous=True)
rospy.Subscriber("/Ego_topic", EgoVehicleStatus, EgoStatus_callback)

# print("_input", _input)
# listener()