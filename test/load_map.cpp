#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "load_map_bag");
  ros::NodeHandle nh;
  std::string topic;
  std::string map_path;
  bool loaded = nh.param<std::string>("vis_topic", topic, "grid_map") &&
                nh.param<std::string>("map_path", map_path, "map.bag");
  if (!loaded){
    ROS_ERROR("Parameters not loaded!");
  }

  ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>(topic, 1);

  grid_map::GridMap map;

  ROS_INFO("Loading map from %s", map_path.c_str());
  grid_map::GridMapRosConverter::loadFromBag(map_path, "grid_map", map);
  // print map layers
  ROS_INFO("Map layers:");
  for (auto& layer : map.getLayers()) {
    ROS_INFO("  %s", layer.c_str());
  }
  // convert the map into a message
  grid_map_msgs::GridMap message;


  Eigen::Vector2d positionOriginMapBag;
  grid_map::Index index = Eigen::Vector2i(0, 0);
  map.getPosition(index, positionOriginMapBag);
  // ROS_INFO_STREAM("BagToMap: positionOriginMapBag: " << positionOriginMapBag);

  bool published = false;
  while (ros::ok()) {
    // publish the map message
    ros::spinOnce();
    // check number of subscribers to the topic
    if (map_pub.getNumSubscribers() > 0 && !published) {
      ROS_INFO_STREAM("[LoadBag]: Publishing map");
      grid_map::GridMapRosConverter::toMessage(map, message);
      map_pub.publish(message);
      published = true;
    }
    if (map_pub.getNumSubscribers() == 0) {
      published = false;
    }
  }
}