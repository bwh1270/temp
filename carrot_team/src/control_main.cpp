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

    bool where = true;
    float current_position[3];  // x, y, z
    float target_poi_yaw[4];    // x, y, z, yaw
    bool willing_to_go = false; // flag which represents vehicle's just moving to target poi
    int obstacle_flag = 0;          // flag 0: can go   1: need to avoid    2: danger

    AIMS::Vehicle carrot_vehicle = AIMS::Vehicle(&nh);
    Target_POI target_poi = Target_POI(&nh);
    Depth depth = Depth(&nh);
    
    ros::Rate rate(1);
    while (ros::ok()) {
        if (where) {
            // Request: I'm here, where to go?
            carrot_vehicle.get_current_pos(current_position);

            // Response: Find the target poi
            target_poi.find_min_range(current_position);
            target_poi.get_target_poi(target_poi_yaw);

            // Set the zoffset & yaw, and I'm ready to move
            carrot_vehicle.set_zoffset_yaw(target_poi_yaw);
            carrot_vehicle.start_moving(&willing_to_go);

            // Now, vehicle starts moving
            if (willing_to_go) {
                while (1) {
                    obstacle_flag = depth.does_obstacle_exist();

                    if (obstacle_flag == 0) {
                        carrot_vehicle.set_xyoffset(target_poi_yaw);
                    }
                    else if (obstacle_flag == 1) {
                        carrot_vehicle.hovering();
                        carrot_vehicle.go_left();
                    }
                    else if (obstacle_flag == 2) {
                        carrot_vehicle.hovering();
                        carrot_vehicle.go_back();
                    }

                    // Vehicle is arrived at the target poi
                    if (carrot_vehicle.arrived(target_poi_yaw)) {
                        break;
                    }
                }
                willing_to_go = false;
            }
            where = false;
        } 
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}