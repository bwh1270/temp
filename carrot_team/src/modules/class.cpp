#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64MultiArray.h"

#include "carrot_team/orientation.hpp"
#include "carrot_team/class.hpp"


Target_POI::Target_POI(ros::NodeHandle *nh) {
    target_idx_ = -1;
    poi_sub_ = nh->subscribe("/red/poi", 100, &Target_POI::poi_sub_callback, this);
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
}

void Target_POI::calculate_range(float *temp_poi, float *current_pos, float *range, float *yaw) {
    float delta_x, delta_y, delta_z;
    delta_x = current_pos[0] - temp_poi[0];
    delta_y = current_pos[1] - temp_poi[1];
    delta_z = current_pos[2] - temp_poi[2];
    *range = (delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z);
    *yaw = atan(delta_x / delta_y);
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



AIMS::Vehicle::Vehicle(ros::NodeHandle *nh) {
    current_position_[0] = 0;
    current_position_[1] = 0;
    current_position_[2] = 0;

    pos_sub_ = nh->subscribe("/red/pose", 100, &AIMS::Vehicle::pos_sub_callback, this);

    zyaw_pub_ = nh->advertise<geometry_msgs::PoseStamped>("/red/tracker/input_pose", 10);
}

void AIMS::Vehicle::pos_sub_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_position_[0] = msg->pose.position.x;
    current_position_[1] = msg->pose.position.y;
    current_position_[2] = msg->pose.position.z;
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

    geometry_msgs::PoseStamped zyaw_pose_msg;
    zyaw_pose_msg.pose.position.z = target_poi_yaw[2];
    zyaw_pose_msg.pose.orientation.w = q.w;
    zyaw_pose_msg.pose.orientation.x = q.x;
    zyaw_pose_msg.pose.orientation.y = q.y;
    zyaw_pose_msg.pose.orientation.z = q.z;
    zyaw_pub_.publish(zyaw_pose_msg);
    ROS_INFO("Setting zoffset and yaw to target");
}


Depth::Depth(ros::NodeHandle *nh) {
    height_ = 640;
    width_  = 480;
    depth_size_ = (640*480);
    for (int i=0; i<depth_size_; ++i) { depth_array_[i] = 0; }
    depth_sub_ = nh->subscribe("/red/camera/depth/image_raw", 1000, &Depth::depth_sub_callback, this);
}

void Depth::depth_sub_callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    for (int i=0; i<depth_size_; ++i) {
        depth_array_[i] = msg->data[i];
    }
    ROS_INFO("(320, 240): [%f]", depth_array_[320*240]);
}
