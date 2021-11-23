#include <ros/ros.h>

Vector2d getMapOrigin(const GridMap& map)
{
    Vector2d origin;
    map.getPosition(Eigen::Vector2i(0, 0), origin);
    return origin;
    }
    

int main(int argc, char **argv)
{
  ros::init(argc, argv, "align_maps_node");
  ros::NodeHandle nh;

  ros::Subscriber mapFromBag = nh.subscribe("grid_map", 1, &traversability::MapToImage::mapCallback, &traversability::mapToImage);
  ros::spin();
}