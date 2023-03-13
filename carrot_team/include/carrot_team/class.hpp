#ifndef CARROT_TEAM_CLASS
#define CARROT_TEAM_CLASS

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64MultiArray.h"

#include "carrot_team/orientation.hpp"

class Target_POI
{
    private:
    ros::Subscriber poi_sub_;
    carrot_team::poi point_of_interests_;

    int target_idx_;
    float target_yaw_;

    public:
    Target_POI(ros::NodeHandle *nh);

    void poi_sub_callback(const carrot_team::poi::ConstPtr &msg);

    void calculate_range(float *temp_poi, float *current_pos, float *range, float *yaw);

    void find_min_range(float *current_pos);

    void get_target_poi(float *target_poi_yaw);

};


namespace AIMS {
    class Vehicle
    {
        private:
        float current_position_[3];    // x,y,z
        float current_orientation_[4]; // x,y,z,w
        ros::Subscriber pos_sub_;
        ros::Publisher zyaw_pub_;
        ros::Publisher xy_pub_;

        public:
        Vehicle(ros::NodeHandle *nh);

        void pos_sub_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void get_current_pos(float *current_position);

        void set_zoffset_yaw(float *target_poi_yaw);

        void start_moving(bool *flag);
        
        void set_xyoffset(float *target_poi_yaw);

        void hovering();

        void go_left();

        void go_right();

        void go_back();

        bool arrived(float *target_poi_yaw);
    };
}


class Depth
{
    private:
    uint height_;
    uint width_;
    uint depth_size_;
    double depth_array_[640*480];
    ros::Subscriber depth_sub_;
    
    public:  
    Depth(ros::NodeHandle *nh);

    void depth_sub_callback(const std_msgs::Float64MultiArray::ConstPtr &msg);

    int does_obstacle_exist();
};
#endif