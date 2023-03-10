# -*- coding: utf-8 -*-
#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO

from sensor_msgs.msg import Image
import cv2


class Detector : 
    
    def __init__(self): 
        self.weights = '../detector/crack_v8_x_500epoch.pt'
        self.model = YOLO(self.weights)
        self.c_cut = 0.5
        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img_display = img.copy()
        # img : 이미지 배열 (480,640,3)
        
        results = self.model.predict(source = img, save = False, show = False, verbose = False)
        
        for result in results: 
            conf_list = result.boxes.conf
            xyxy_list = result.boxes.xyxy
            xywh_list = result.boxes.xywh_list
            
        conf = conf_list.cpu().detach().numpy()
        
        if len(conf) != 0: 
            idx = np.where(conf > self.c_cut)
            
            for xyxy in xyxy_list[idx] : 
                cv2.rectangle(img_display, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), thickness = 3)
                
            xywh = xywh_list[idx].cpu().detach().numpy()
            
        cv2.imshow('detection result', img_display)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('yolo_ros', anonymous=True)
    detector = Detector()
    rospy.Subscriber('/red/camera/color/image_raw', Image, detector.image_callback)
    rospy.spin()