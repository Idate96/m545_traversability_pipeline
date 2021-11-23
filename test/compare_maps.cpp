#include <ros/ros.h>
#include "ImageToMap.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_compare_node");
    ros::NodeHandle nh;
    traversability::ImageToMap image_to_map(nh);

    // load bag
    grid_map::GridMap mapFromBag;
    grid_map::GridMapRosConverter::loadFromBag("occupancy_map.bag", "grid_map", mapFromBag);

    // publish both bags in rviz to compare 
    grid_map_msgs::GridMap messageFromBag;
    grid_map::GridMapRosConverter::toMessage(mapFromBag, messageFromBag);
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map_bag", 1);

    while( ros::ok() ){
        map_pub.publish(messageFromBag);
        ros::spinOnce();
        ros::Duration(10).sleep();
    }
}