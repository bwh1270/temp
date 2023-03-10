#!/usr/bin/env python

import os # mkdir
import sys # check python version

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge # Change ros image into opencv
from sensor_msgs.msg import Image # Subscribe image
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_msgs.msg import String

# distance publisher
pub_d = rospy.Publisher('/carrot_team/distance', Float64 , queue_size = 10)
pub_lr = rospy.Publisher('/carrot_team/lr', String, queue_size = 10)
pub_ud = rospy.Publisher('/carrot_team/ud', String, queue_size = 10)
pub_distance = rospy.Publisher('/carrot_team/distance_array', Float64MultiArray, queue_size = 10)

bridge = CvBridge() # Get drone image
bridge_2 = CvBridge() # Get drone depth map

img = np.array([]) # 전역변수

# 동영상 저장하는 폴더가 없으면 만들기
base_dir = '/root/video'

if not os.path.isdir(base_dir):
    os.mkdir(base_dir)

# 동영상 저장 경로
vid_name = base_dir + '/drone_vid.avi'
vid_name_d = base_dir + '/drone_vid_depth.avi'

# 동영상 코덱
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter(vid_name, fourcc, 30.0, (640, 480))
out_d = cv2.VideoWriter(vid_name_d, fourcc, 30.0, (640, 480))


def image_callback(msg):
    global img
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # cv2.imshow('Drone image', img)

    # cv2.waitKey(25)


def image_callback_depth(msg):
    global img
    
    # get depth image
    img_depth_ori = bridge_2.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img_depth = img_depth_ori / 10 # convert into (유사) grayscale

    ### send distance
    # 전방에 얼마나 남았는지
    h, w = np.shape(img_depth_ori)
    w_half = w // 2
    h_half = h // 2
    
    dist_mid = img_depth_ori[h_half][w_half]
    dist_pub = 0

    isNaN = np.isnan(dist_mid)
    if isNaN:
        dist_pub = 10
    else:
        dist_pub = dist_mid
    
    
    # publish distance data
    img_data = img_depth_ori * 1
    
    img_data[np.isnan(img_data)] = 10.0

    img_array = Float64MultiArray()

    dim1 = MultiArrayDimension()
    dim1.label = "height"
    dim1.size = img_data.shape[0]
    dim1.stride = img_data.shape[1]

    dim2 = MultiArrayDimension()
    dim1.label = "width"
    dim1.size = img_data.shape[1]
    dim2.siride = 1

    img_array.layout.dim.append(dim1)
    img_array.layout.dim.append(dim2)

    img_array.data = img_data.ravel().tolist()

    pub_distance.publish(img_array)
    
    # publish distance
    pub_d.publish(dist_pub)

    
    # 좌우 중에 어디가 더 많이 남았는지
    left = img_depth_ori[h_half][:w_half]
    right = img_depth_ori[h_half][w_half:]

    sum_l = 0
    for i in left:
        if np.isnan(i):
            sum_l += 10
        else:
            sum_l += i

    sum_r = 0
    for i in right:
        if np.isnan(i):
            sum_r += 10
        else:
            sum_r += i

    if sum_l > sum_r:
        pub_lr.publish('l')
    else:
        pub_lr.publish('r')

    # 상하 중에 어디가 더 많이 남았는지 - 맞는지 확인 위가 0이고 아래가 480이면 맞고, 아니면 반대로 바꿔야 된다.
    up = img_depth_ori[:h_half, w_half]
    down = img_depth_ori[h_half:, w_half]

    sum_u = 0
    for i in up:
        if np.isnan(i):
            sum_u += 10
        else:
            sum_u += i

    sum_d = 0
    for i in down:
        if np.isnan(i):
            sum_d += 10
        else:
            sum_d += i

    if sum_u > sum_d:
        pub_ud.publish('u')
    else:
        pub_ud.publish('d')

    ### send distance end

    # Record
    global out
    global out_d

    out.write(img)
    # out_d.write(img_depth) # 코덱이 안맞아서 저장이 안되는 것 같다.

    # imshow
    cv2.imshow('Depth image', img_depth)
    cv2.imshow('Drone image', img)

    cv2.waitKey(25)


# roslaunch 할때랑 rosrun 할때랑 파이썬 버전이 다른거 같아서 확인
sys.version
print('OpenCv version')
cv2.__version__

rospy.init_node('image_subscriber', anonymous=True)
rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)
rospy.Subscriber('/red/camera/depth/image_raw', Image, image_callback_depth)
rospy.spin()

# 이거는 안쓰일거 같다. - 수정
cv2.destroyAllWindows()
out.release()
out_d.release()

# get py code
# wget https://raw.githubusercontent.com/yijh0611/HILFIGER/main/src/carrot_team/src/sub_img.py
# chmod +x sub_img.py