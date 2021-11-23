#include <ros/ros.h>
#include "ImageToMap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_map");
    ros::NodeHandle nh;
    traversability::ImageToMap image_to_map(nh);
    ros::spin();
    // ros rate is 0.1 Hz
    ros::Rate(0.1);
    return 0;
}
