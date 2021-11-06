#pragma once

// ROS
#include <ros/ros.h>

#include <string>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace traversability {

/*!
 * Saves an ElevationMapping layer as image.
 */
class MapToImage {
 public:
  MapToImage(ros::NodeHandle& nodeHandle);
  ~MapToImage() = default;
 private:
  /*!
   * @brief Reads and verifies the ROS parameters.
   */
  void readParameters();

  /**
   * @brief The callback receiving the grid map.
   * It will convert the specified layer into a png image and save it to the specified location. Afterwards the node will terminate.
   * @param msg the recieved grid map to save to a file.
   */
  void gridMapCallback(const grid_map_msgs::GridMap& msg);

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! GridMap subscriber
  ros::Subscriber gridMapSubscriber_;

  //! Name of the grid map topic.
  std::string gridMapTopic_;
  std::string gridMapLayer_;

  //! Path where to store the image.
  std::string filePath_;
};

}  