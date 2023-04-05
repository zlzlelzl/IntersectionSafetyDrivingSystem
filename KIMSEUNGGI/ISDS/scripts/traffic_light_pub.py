#!/usr/bin/env python3.7
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from morai_msgs.msg import TrafficSignInfo
from morai_msgs.msg import TrafficSign
import torch
import cv2
import numpy as np
import os, rospkg
from PIL import Image

import warnings
warnings.simplefilter("ignore", DeprecationWarning)

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self, pkg_name = 'ssafy_3'):
        self.model = torch.hub.load('/home/kimsngi/catkin_ws/src/ssafy_2/scripts/yolov5/', 'custom', path='/home/kimsngi/catkin_ws/src/ssafy_2/scripts/yolov5/best.pt', source='local', force_reload=True)
        
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.traffic_sign_info_pub = rospy.Publisher('/traffic_sign_info', TrafficSignInfo , queue_size=10)

        self.img_rgb = None
        rate = rospy.Rate(30) # 1 hz
        while not rospy.is_shutdown():
            #result array to save topic msg
            #print(self.img_bgr)
            if self.img_rgb is not None:
                msg = []

                # image frame yolov5 model processing
                #traffic_sign_info_raw = process_yolov5_model('/home/ssafy/catkin_ws/src/ssafy_3/scripts/yolov5/test.jpg')
                output = self.model(self.img_rgb)
                
                names = output.pandas().xyxy[0].name
        
                #filtering type, value of traffic light information
                for n in names:
                    #print(n)
                    sign = self.generate_msg_topic(n)
                    msg.append(sign)
        
                #publish topic

                # print(msg)
                self.traffic_sign_info_pub.publish(msg)
                self.drawLabel(self.img_rgb, output)
                #output.pandas().show()
                

                # cv2.waitKey(1)
            rate.sleep()

    def callback(self, msg):
        #print(msg)
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #self.img_rgb = cv2.cvtColor(self.img_bgr, cv2.COLOR_RGB2BGR)
            # print(self.img_bgr)
            ww = 640
            hh = 480
            # self.img_bgr = self.img_bgr[0:250,0:250] 
            # self.img_bgr = self.img_bgr[int(1/4 * ww):int(3/4  * ww),int(0 * hh):int(1 * hh)] 
            self.img_rgb = self.img_rgb[:, 120:480] 
            # cv2.imshow("origin_img", self.img_bgr)
            # cv2.imshow("origin_img2", self.img_bgr2)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)


    def generate_msg_topic(self, string):
        #filtering from ignOOOO form
        type = int(string[4:6])
        value = int(string[6:])
    
        traffic_sign_msg = TrafficSign()
        traffic_sign_msg.traffic_light_type = type
        traffic_sign_msg.traffic_light_status = value

        return traffic_sign_msg
    
    def drawLabel(self, img, obj):
        shapes = np.zeros_like(img, np.uint8)
        out = img.copy()
        for i in range(len(obj.pandas().xyxy[0])):
        
            x1 = int(obj.pandas().xyxy[0].xmin[i])
            y1 = int(obj.pandas().xyxy[0].ymin[i])
            x2 = int(obj.pandas().xyxy[0].xmax[i])
            y2 = int(obj.pandas().xyxy[0].ymax[i])
            conf = obj.pandas().xyxy[0].confidence[i]
            name = obj.pandas().xyxy[0].name[i]
            #print(x1,y1,x2,y2,conf, name)
            cv2.rectangle(shapes, (x1, y1),(x2, y2),(102,255,204),-1)

            alpha = 0.7
            mask = shapes.astype(bool)
            out[mask] = cv2.addWeighted(img, 0.5, shapes, 1-alpha, 0)[mask]
            cv2.putText(out, str(name)+'  '+str(conf), (x1, y1), cv2.FONT_HERSHEY_PLAIN, 1, (102,255,204))
        cv2.imshow('result', out)
        cv2.waitKey(1)

    def show_frame(self, output): 
        # Display frames in main program
        arr = output.datah().cpu().numpy()
        img = Image.fromarray.fromarray(arr, 'RGB')
        result = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        cv2.imshow('frame', result)
        key = cv2.waitKey(1)


if __name__ == '__main__':
  
    rospy.init_node('traffic_sign_publisher', anonymous=True)

    image_parser = IMGParser()
    rospy.spin()
