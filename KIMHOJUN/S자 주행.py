#!/usr/bin/env python

import rospy
from morai_msgs.msg import CtrlCmd

class s_drive():
    def __init__(self):
        rospy.init_node('s_drvie', anonymous=True)
        cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        rate = rospy.Rate(45)
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.velocity = 10
        steering_cmd = [-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5]
        cmd_cnts = 50

        while not rospy.is_shutdown():
            for i in range(11):
                cmd.steering = steering_cmd[i]
                rospy.loginfo(cmd)
                for _ in range(cmd_cnts):
                    cmd_pub.publish(cmd)
                    rate.sleep()
            for i in range(11)[::-1]:
                cmd.steering = steering_cmd[i]
                rospy.loginfo(cmd)
                for _ in range(cmd_cnts):
                    cmd_pub.publish(cmd)
                    rate.sleep()



if __name__ == '__main__':
    try:
        s_d = s_drive()
    except rospy.ROSInterruptException:
        pass