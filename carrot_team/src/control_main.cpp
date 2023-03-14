#include <iostream>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"

#include "carrot_team/orientation.hpp"
#include "carrot_team/class.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_main");
    ros::NodeHandle nh;

    bool next_poi = false;
    float current_position[3];  // x, y, z
    float target_poi_yaw[4];    // x, y, z, yaw
    bool willing_to_go = false; // flag which represents vehicle's just moving to target poi
    int obstacle_flag = 0;          // flag 0: can go   1: need to avoid    2: danger

    Target_POI target_poi = Target_POI(&nh);
    ros::Rate _rate(1);
    while (ros::ok()) {
        target_poi.start(&next_poi);
        if (next_poi) { break; }
        ros::spinOnce();
        _rate.sleep();
    }
    AIMS::Vehicle carrot_vehicle = AIMS::Vehicle(&nh);
    Depth depth = Depth(&nh);
    for (int i=0; i<10; ++i) { ros::spinOnce(); _rate.sleep(); }
    
    ros::Rate rate(1);
    while (ros::ok()) 
    {
        if (next_poi) 
        {
            // Request: I'm here, where to go?
            carrot_vehicle.get_current_pos(current_position);

            // Response: Find the target poi
            target_poi.find_min_range(current_position);
            target_poi.get_target_poi(target_poi_yaw);

            // Set the zoffset & yaw, and I'm ready to move
            carrot_vehicle.set_zoffset_yaw(target_poi_yaw);
            carrot_vehicle.start_moving(&willing_to_go);

            // Now, vehicle starts moving
            if (willing_to_go) 
            {
                bool obs_free = false;
                while (1) 
                {
                    //depth.does_obstacle_exist(&obstacle_flag);
                    
                    // experiment-1: 첫 번째 poi까지 도달
                    // obstacle_free()를 통해 flag가 false에서 true로 변환
                    obs_free = true;
                    if (obs_free) {
                        carrot_vehicle.set_xyoffset();
                    }

                    if (carrot_vehicle.arrived(target_poi_yaw)) {
                        willing_to_go = false;
                        break;
                    }
                }
                // temp stopper
                next_poi = false;
            }

        ros::spinOnce();
        rate.sleep();
        }
    }
    return 0;
}
