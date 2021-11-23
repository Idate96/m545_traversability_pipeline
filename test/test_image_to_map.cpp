#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ImageToMap.h>
#include <grid_map_ros/grid_map_ros.hpp>


TEST(ImageToMap, test_image_to_map_frames){
    ros::init(argc, argv, "image_to_map");
    ros::NodeHandle nh;
    traversability::ImageToMap image_to_map(nh);

    // load bag
    grid_map::GridMap mapFromBag;
    grid_map::GridMapRosConverter::loadFromBag("occupancy_map.bag", "grid_map", mapFromBag);
    
    // publish both bags in rviz to compare 
    grid_map_msgs::GridMap messageFromBag;
    grid_map::GridMapRosConverter::toMessage(mapFromBag, messageFromBag);
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map_bag", 1);
    map_pub.publish(messageFromBag);

    ros::spin();
    // ros rate is 0.1 Hz
    ros::Rate loop_rate(10);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}