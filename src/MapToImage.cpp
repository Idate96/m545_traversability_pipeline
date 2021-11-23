#include "MapToImage.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>

namespace traversability {

MapToImage::MapToImage(ros::NodeHandle& nh): nodeHandle_(nh){
  readParameters();
  gridMapSubscriber_ = nodeHandle_.subscribe(gridMapTopic_, 1, &MapToImage::gridMapCallback, this);
}

void MapToImage::readParameters(){
  nodeHandle_.param("grid_map_topic", gridMapTopic_, std::string("/grid_map"));
  nodeHandle_.param("file_path", filePath_, std::string("data/map.bag"));
  nodeHandle_.param("layer", gridMapLayer_, std::string("elevation"));
}

void MapToImage::gridMapCallback(const grid_map_msgs::GridMap& msg){
  ROS_INFO("Saving map received from: %s to file %s.", nodeHandle_.resolveName(gridMapTopic_).c_str(), filePath_.c_str());
  grid_map::GridMap map;
  cv_bridge::CvImage image;
  grid_map::GridMapRosConverter::fromMessage(msg, map, {gridMapLayer_});
  grid_map::GridMapRosConverter::toCvImage(map, gridMapLayer_, sensor_msgs::image_encodings::MONO8, image);
  bool success = cv::imwrite(filePath_.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
  ROS_INFO("Success writing image: %s", success?"true":"false");
  ros::shutdown(); 
}

}