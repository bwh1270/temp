#! /usr/bin/env python

import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge    # ros image -> opencv
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

def depth_callback(msg):
    global cv_depth
    cv_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
    assert cv_depth.size == (640*480)
    depth_1D = np.ravel(cv_depth)
    print(depth_1D)
    array_msg = Float64MultiArray()
    array_msg.data = list(depth_1D)
    depth_pub.publish(array_msg)

    cv2.imshow("Depth_image", cv_depth)
    cv2.waitKey(20)



if __name__ == "__main__":
    
    bridge = CvBridge();
    cv_depth = np.empty(shape=[0]);

    rospy.init_node("get_img", anonymous=False)
    depth_pub = rospy.Publisher("/carrot_team/depth_array", Float64MultiArray, queue_size = 10)
    rospy.Subscriber("/red/camera/depth/image_raw", Image, depth_callback)
    rospy.spin()