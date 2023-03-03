#!usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import CtrlCmd, CollisionData, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from enum import Enum

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4

class s_drive():
    def __init__(self):
        rospy.init_mode("collision_avoid", anonymous=True)

        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        rospy.Subscriber('/CollisionData', CollisionData, self.collision_callback)
        rospy.Subscriber('Ego_topic', EgoVehicleStatus, self.ego_callback)

        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.rate = rospy.Rate(10)

        self.is_collision = False
        self.ego_status = EgoVehicleStatus()

        self.send_gear_cmd(Gear.D.value)
        
        while not rospy.is_shutdown():
            if (self.is_collision):
                self.send_gear_cmd(Gear.R.value)

                for _ in range(20):
                    self.send_ctrl_cmd(0.4, 10)
                    self.rate.sleep()
                
                self.send_gear_cmd(Gear.D.value)

            else:
                self.send_ctrl_cmd(0, 10)
                self.rate.sleep()

    def collision_callback(self, data):
        if(len(data.collision_object) > 0):
            self.is_Collision = True
        else:
            self.is_collision = False
        
    
    def ego_callback(self, data):
        print(self.ego_status.velocity.x)
        self.ego_status = data

    def send_gear_cmd(self, gear_mode):
        while (abs(self.ego_status.velocity.x) > 0.1):
            self.send_ctrl_cmd(0, 0)
            self.rate.sleep()
        
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(gear_cmd)

    def send_ctrl_cmd(self, steering, velocity):
        cmd = CtrlCmd()
        if(velocity > 0):
            cmd.longCmdType = 2
            cmd.velocity = velocity
            cmd.steering = steering
        else:
            cmd.longlCmdType = 1
            cmd.brake = 1
            cmd.steering = 0
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass