#include "ros/ros.h"
#include "carrot_team/poi.h"

void poi_sub_callback(const carrot_team::poi::ConstPtr &msg) {
    carrot_team::poi poi_arr;
    int poi_len = msg->poi.size();
    poi_arr.poi.resize(poi_len);

    for (int i=0; i<poi_len; ++i) {
        poi_arr.poi[i].x = msg->poi[i].x;
        poi_arr.poi[i].y = msg->poi[i].y;
        poi_arr.poi[i].z = msg->poi[i].z;
    }

    poi_pub.publish(poi_arr);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "poi_talker");
    ros::NodeHandle nh;

    bool should_latch = true;
    ros::Publisher poi_pub = nh.advertise<carrot_team::poi>("/carrot_team/poi", 10, should_latch);

    ros::Subscriber poi_sub = nh.subscribe("/red/poi", 1000, &poi_sub_callback);
    ros::spin();
}
