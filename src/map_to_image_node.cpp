#include <ros/ros.h>
#include "MapToImage.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_to_image_node");
  ros::NodeHandle nh;
  traversability::MapToImage mapToImage(nh);
  ros::spin();
  return 0;
}