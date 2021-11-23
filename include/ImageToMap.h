#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "message_filters/synchronizer.h"
#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

namespace traversability {

/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class ImageToMap
{
 public:
  ImageToMap(ros::NodeHandle& nodeHandle);
  ~ImageToMap() = default;

  bool readParameters();

  void imageCallback(const sensor_msgs::ImageConstPtr& elevation_msg, 
                     const sensor_msgs::ImageConstPtr& occupancy_msg);
  void publishMap();
  void saveMap();

 private:
  ros::NodeHandle& nodeHandle_;
  ros::Publisher gridMapPublisher_;

  message_filters::Subscriber<sensor_msgs::Image> imageElevationSubscriber_;
  message_filters::Subscriber<sensor_msgs::Image> imageOccupancySubscriber_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> syncPtr_;
  
  grid_map::GridMap map_;
  
  std::string imageElevationTopic_;
  std::string imageOccupancyTopic_;
  double mapLengthX_;
  double resolution_;
  double minHeight_;
  double maxHeight_;
  double xOffset_;
  double yOffset_;
  Eigen::Vector2d mapOrigin_;
  std::string mapFrameId_;
  std::string savePath_;

  bool mapInitialized_ = false;
  bool published_ = false;
  bool savedMap_ = false;
};

} /* namespace */
