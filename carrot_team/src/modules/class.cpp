#include <iostream>
#include <vector>
#include <cmath>
#ifndef _WIN32
#include <unistd.h>	 
#else
#include <windows.h>
#endif

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32MultiArray.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

#include "carrot_team/orientation.hpp"
#include "carrot_team/class.hpp"


Target_POI::Target_POI(ros::NodeHandle *nh) {
    target_idx_ = -1;
    point_of_interests_.poi.resize(10);
    poi_sub_ = nh->subscribe("/carrot_team/poi", 100, &Target_POI::poi_sub_callback, this);
}

void Target_POI::poi_sub_callback(const carrot_team::poi::ConstPtr &msg) {
    /* Temporary poi -> permanent poi */
    int poi_len = msg->poi.size();
    point_of_interests_.poi.resize(poi_len);

    int _i;
    for (_i = 0; _i < poi_len; ++_i) {
        point_of_interests_.poi[_i].x = msg->poi[_i].x;
        point_of_interests_.poi[_i].y = msg->poi[_i].y;
        point_of_interests_.poi[_i].z = msg->poi[_i].z;
    } 
    std::cout << "# of poi: " <<  point_of_interests_.poi.size() << std::endl;
    ROS_INFO("Successfully save poi");
}

void Target_POI::start(bool *flag) {
    if (point_of_interests_.poi[0].x != 0) {
        *flag = true;
        std::cout << "Successfully saving poi" << std::endl;
    }
    else {
        std::cout << "Still waiting for saving poi" << std::endl;
    }
}

void Target_POI::calculate_range(float *temp_poi, float *current_pos, float *range, float *yaw) {
    float delta_x, delta_y, delta_z;
    delta_x = current_pos[0] - temp_poi[0];
    delta_y = current_pos[1] - temp_poi[1];
    delta_z = current_pos[2] - temp_poi[2];
    *range = (delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z);

    float _c_vec_amount = sqrt(current_pos[0]**2 + current_pos[1]**2 + current_pos[2]**2);
    float _t_vec_amount = sqrt(temp_poi[0]**2 + temp_poi[1]**2 + temp_poi[2]**2);
    float _dot_product = current_pos[0]*temp_poi[0] + current_pos[1]*temp_poi[1] + current_pos[2]*temp_poi[2];
    *yaw = acos(_dot_product / (_c_vec_amount * _t_vec_amount));
}

void Target_POI::find_min_range(float *current_pos) {
    /* find minimum range among poi */
    float range;
    float min_range = 10e9f;

    int len = point_of_interests_.poi.size();
    float temp_poi[3] = {0, 0, 0};
    int _i;
    for (_i=0; _i<len; ++_i) {
        temp_poi[0] = point_of_interests_.poi[_i].x;
        temp_poi[1] = point_of_interests_.poi[_i].y;
        temp_poi[2] = point_of_interests_.poi[_i].z;
        Target_POI::calculate_range(temp_poi, current_pos, &range, &target_yaw_);

        if (min_range >= range) {
            min_range = range;
            target_idx_ = _i;
        }
    }
    ROS_INFO("target index of poi is [%d]", target_idx_);
    ROS_INFO("target yaw of poi is [%f]", target_yaw_);
}

void Target_POI::get_target_poi(float *target_poi_yaw) {
    /* send the target poi to outside */
    if (target_idx_ >= 0) {
        target_poi_yaw[0] = point_of_interests_.poi[target_idx_].x;
        target_poi_yaw[1] = point_of_interests_.poi[target_idx_].y;
        target_poi_yaw[2] = point_of_interests_.poi[target_idx_].z;
        target_poi_yaw[3] = target_yaw_;
    }
    else {
        ROS_INFO("target idx is not defined yet");
    }
}

void Target_POI::possible_points() {
    float poi_x, poi_y, poi_z;
    float c_theta = 3.14159265358979323846 / 4;
    int z_offset[2] = {2, 4};
    pp_.poi.resize(8*2);
    int cnt = 0;

    // 16 points
    poi_x = point_of_interests_.poi[target_idx_].x;
    poi_y = point_of_interests_.poi[target_idx_].y;
    poi_z = point_of_interests_.poi[target_idx_].z;

    for (int i=0; i<8; ++i) {
        pp_.poi[cnt].x = poi_x + 2*cos(c_theta*i);
        pp_.poi[cnt].y = poi_y + 2*sin(c_theta*i); 
        for (int j=0; j<2; ++j) {
            pp_.poi[cnt].z = poi_z + z_offset[j];
            ++cnt;
        }
    }   
}

// 현재 위치 고려해서 possible_points 중에 가야할 순서 정해주는 함수

// mapping 고려해서 possible_points 중에 가지 못하는 점 삭제하는 함수


AIMS::Vehicle::Vehicle(ros::NodeHandle *nh) {
    for (int i=0; i<3; ++i) { current_position_[i] = 0; }
    for (int i=0; i<4; ++i) { current_orientation_[i] = 0;}

    pos_sub_ = nh->subscribe("/red/pose", 100, &AIMS::Vehicle::pose_sub_callback, this);

    zyaw_pub_ = nh->advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/red/position_hold/trajectory", 100);
    xy_pub_   = nh->advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/red/position_hold/trajectory", 100);
}

void AIMS::Vehicle::pose_sub_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_position_[0] = msg->pose.position.x;
    current_position_[1] = msg->pose.position.y;
    current_position_[2] = msg->pose.position.z;
    current_orientation_[0] = msg->pose.orientation.x;
    current_orientation_[1] = msg->pose.orientation.y;
    current_orientation_[2] = msg->pose.orientation.z;
    current_orientation_[3] = msg->pose.orientation.w;
}

void AIMS::Vehicle::get_current_pos(float *current_position) {
    for (int i=0; i<3; ++i) {
        current_position[i] = current_position_[i];
    }
}

void AIMS::Vehicle::set_zoffset_yaw(float *target_poi_yaw) {

    EulerAngles angle;
    Quaternion q;
    angle.yaw = target_poi_yaw[3];
    q = ToQuaternion(angle);
    while (1) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint zyaw_pose_msg;
		zyaw_pose_msg.transforms.resize(1);
		zyaw_pose_msg.velocities.resize(1);
		zyaw_pose_msg.accelerations.resize(1);

        zyaw_pose_msg.transforms[0].translation.x = current_position_[0];
        zyaw_pose_msg.transforms[0].translation.y = current_position_[1];
        zyaw_pose_msg.transforms[0].translation.z = target_poi_yaw[2];
        zyaw_pose_msg.transforms[0].rotation.w = q.w;
        zyaw_pose_msg.transforms[0].rotation.x = q.x;
        zyaw_pose_msg.transforms[0].rotation.y = q.y;
        zyaw_pose_msg.transforms[0].rotation.z = q.z;
        zyaw_pub_.publish(zyaw_pose_msg);
        sleep(0.001); // second
        if (abs(current_position_[2] - target_poi_yaw[2]) < 0.01) { // 0.001 scale에서는 일치하지 않음
            break;
        }
    }
    ROS_INFO("Setting zoffset and yaw to target");
}

void AIMS::Vehicle::start_moving(bool *flag) {
    *flag = true;
}

void AIMS::Vehicle::set_xyoffset(float *target_poi_yaw) {

    geometry_msgs::PoseStamped xy_pose_msg;
    xy_pose_msg.pose.position.x = target_poi_yaw[0];
    xy_pose_msg.pose.position.x = target_poi_yaw[1];
    xy_pose_msg.pose.position.x = target_poi_yaw[2];
    xy_pub_.publish(xy_pose_msg);
    ROS_INFO("Setting x,y offset");
}

void AIMS::Vehicle::hovering() {
    geometry_msgs::PoseStamped xyz_pose_msg;
    xyz_pose_msg.pose.position.x = current_position_[0];
    xyz_pose_msg.pose.position.y = current_position_[1];
    xyz_pose_msg.pose.position.z = current_position_[2];
    xy_pub_.publish(xyz_pose_msg);
    ROS_INFO("hovering");
}

void AIMS::Vehicle::go_left() {
    Quaternion _q;
    EulerAngles _angle;
    float left_vec_x, left_vec_y;
    _q.x = current_orientation_[0];
    _q.y = current_orientation_[1];
    _q.z = current_orientation_[2];
    _q.w = current_orientation_[3];

    // move left about 1m w.r.t current orientation 
    _angle = ToEulerAngles(_q); 
    left_vec_x = cos(_angle.yaw) * 0 - sin(_angle.yaw) * 1;
    left_vec_y = sin(_angle.yaw) * 0 + cos(_angle.yaw) * 1;

    geometry_msgs::PoseStamped xy_pose_msg;
    xy_pose_msg.pose.position.x = current_position_[0] + left_vec_x;
    xy_pose_msg.pose.position.y = current_position_[1] + left_vec_y;
    xy_pub_.publish(xy_pose_msg);
    ROS_INFO("go left");
}

void AIMS::Vehicle::go_right() {
    Quaternion _q;
    EulerAngles _angle;
    float right_vec_x, right_vec_y;
    _q.x = current_orientation_[0];
    _q.y = current_orientation_[1];
    _q.z = current_orientation_[2];
    _q.w = current_orientation_[3];

    // move left about 1m w.r.t current orientation 
    _angle = ToEulerAngles(_q); 
    right_vec_x = cos(_angle.yaw) * 0 - sin(_angle.yaw) * (-1);
    right_vec_y = sin(_angle.yaw) * 0 + cos(_angle.yaw) * (-1);

    geometry_msgs::PoseStamped xy_pose_msg;
    xy_pose_msg.pose.position.x = current_position_[0] + right_vec_x;
    xy_pose_msg.pose.position.y = current_position_[1] + right_vec_y;
    xy_pub_.publish(xy_pose_msg);
    ROS_INFO("go right");
}

void AIMS::Vehicle::go_back() {
    Quaternion _q;
    EulerAngles _angle;
    float back_vec_x, back_vec_y;
    _q.x = current_orientation_[0];
    _q.y = current_orientation_[1];
    _q.z = current_orientation_[2];
    _q.w = current_orientation_[3];

    // move left about 1m w.r.t current orientation 
    _angle = ToEulerAngles(_q); 
    back_vec_x = cos(_angle.yaw) * (-1) - sin(_angle.yaw) * (0);
    back_vec_y = sin(_angle.yaw) * (-1) + cos(_angle.yaw) * (0);

    geometry_msgs::PoseStamped xy_pose_msg;
    xy_pose_msg.pose.position.x = current_position_[0] + back_vec_x;
    xy_pose_msg.pose.position.y = current_position_[1] + back_vec_y;
    xy_pub_.publish(xy_pose_msg);
    ROS_INFO("go right");
}

bool AIMS::Vehicle::arrived(float *target_poi_yaw) {
    float delta_x, delta_y;
    delta_x = abs(current_position_[0] - target_poi_yaw[0]);
    delta_y = abs(current_position_[1] - target_poi_yaw[1]);

    if ((delta_x < 0.01) && (delta_y < 0.01)) {
        this->hovering();
        // hovering();
        ROS_INFO("Vehicle is arrived at the target POI");
        // publish (iter, accurate or near, poi(x,y,z))
        return true;
    }
    else { return false; }
}


Depth::Depth(ros::NodeHandle *nh) {
    height_ = 640;
    width_  = 480;
    depth_size_ = (640*480);
    for (int i=0; i<depth_size_; ++i) { depth_array_[i] = 0; }
    depth_sub_ = nh->subscribe("/carrot_team/depth_array", 1000, &Depth::depth_sub_callback, this);  
}

void Depth::depth_sub_callback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    for (int i=0; i<depth_size_; ++i) {
        depth_array_[i] = msg->data[i];
    }
    ROS_INFO("(320, 240): [%f]", depth_array_[320*240]);
}

void Depth::does_obstacle_exist(int *obstacle_flag) {
    int vertical_index_arr[7] = {120, 145, 165, 180, 240, 300, 360};
    int idx = 0;

    for (int i=0; i<7; ++i) {
        for (int j=0; j<7; ++j) {
            if (i==3) {
                if ((j>=1) && (j<=5)) {
                    idx = (100*j+1) + ((vertical_index_arr[i]-1)*639);
                }
            }    
            else {
                idx = (100*j+1) * vertical_index_arr[i];
            }
            
            // 0: you can go   1: need to avoid   2: danger
            if (depth_array_[idx] < 0.3) {
                *obstacle_flag = 2;
            }
            else if (depth_array_[idx] < 1) {
                *obstacle_flag = 1;
            }
            else {
                *obstacle_flag = 0;
            }
        }
    }
}