#!/usr/bin/env python

import time
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

# Initialize the ROS node
rospy.init_node('control', anonymous = True)

# Create a publisher object
pose_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)

# Create a PoseStamped message
pose_msg = PoseStamped()
# pose_msg.header.frame_id = ""  # set the frame ID
pose_msg.pose.position.x = 10.0  # set the x position 범위 0 ~ 15
pose_msg.pose.position.y = 2.0  # set the y position 범위 0 ~ 50
pose_msg.pose.position.z = 2.0  # set the z position 범위 0 ~ 15
pose_msg.pose.orientation.x = 0.0  # set the x orientation
pose_msg.pose.orientation.y = 0.0  # set the y orientation
pose_msg.pose.orientation.z = 0.0  # set the z orientation
pose_msg.pose.orientation.w = 1.0  # set the w orientation

yaw = 0

# Publish the message
while not rospy.is_shutdown():    
    # 다른 제어 방법
    comm = input('Type WASD or IJKL and enter')

    res = 0.5 # 얼마나 이동하는지
    res_yaw = 45
    if comm == 'l':
        # y 방향 이동
        pose_msg.pose.position.x += res  # set the y position
        if pose_msg.pose.position.x > 14:
            pose_msg.pose.position.x = 14

    elif comm == 'j':
        pose_msg.pose.position.x -= res
        if pose_msg.pose.position.x < 1:
            pose_msg.pose.position.x = 1
    
    elif comm == 'k':
        pose_msg.pose.position.y -= res
        if pose_msg.pose.position.y < 1:
            pose_msg.pose.position.y = 1
    
    elif comm == 'i':
        pose_msg.pose.position.y += res
        if pose_msg.pose.position.y > 49:
            pose_msg.pose.position.y = 49

    elif comm == 'w':
        pose_msg.pose.position.z += res
        if pose_msg.pose.position.z > 24:
            pose_msg.pose.position.z = 24

    elif comm == 's':
        pose_msg.pose.position.z -= res
        if pose_msg.pose.position.z < 1:
            pose_msg.pose.position.z = 1
        
    elif comm == 'a':
        yaw += res_yaw

        q = get_quaternion_from_euler(0, 0, yaw)

        pose_msg.pose.orientation.x = q[0]  # set the x orientation
        pose_msg.pose.orientation.y = q[1]  # set the y orientation
        pose_msg.pose.orientation.z = q[2]  # set the z orientation
        pose_msg.pose.orientation.w = q[3]  # set the w orientation

    elif comm == 'd':
        yaw -= res_yaw

        q = get_quaternion_from_euler(0, 0, yaw)

        pose_msg.pose.orientation.x = q[0]  # set the x orientation
        pose_msg.pose.orientation.y = q[1]  # set the y orientation
        pose_msg.pose.orientation.z = q[2]  # set the z orientation
        pose_msg.pose.orientation.w = q[3]  # set the w orientation
    
    else:
        print('Enter WASD or IJKL')

    # move drone
    print('move to : ', pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
    pose_publisher.publish(pose_msg)