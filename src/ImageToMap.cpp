#include "ImageToMap.h"
#include <boost/bind.hpp>
#include <functional>

namespace traversability
{
    ImageToMap::ImageToMap(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle),
                                                          map_(grid_map::GridMap({"elevation", "occupancy"}))
    {
        if (!readParameters())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("edited_grid_map", 1);
        imageElevationSubscriber_.subscribe(nodeHandle_, imageElevationTopic_, 1);
        imageOccupancySubscriber_.subscribe(nodeHandle_, imageOccupancyTopic_, 1);
        syncPtr_.reset(new Sync(MySyncPolicy(10), imageElevationSubscriber_, imageOccupancySubscriber_));
        syncPtr_->registerCallback(boost::bind(&ImageToMap::imageCallback, this, _1, _2));
    }

    bool ImageToMap::readParameters()
    {
        bool success = nodeHandle_.param("image_elevation_topic", imageElevationTopic_, std::string("image/elevation")) &&
                       nodeHandle_.param("image_occupancy_topic", imageOccupancyTopic_, std::string("image/occupancy")) &&
                       nodeHandle_.param("/image_to_map/resolution", resolution_, 0.2) &&
                       nodeHandle_.param("/image_to_map/min_height", minHeight_, 0.0) &&
                       nodeHandle_.param("/image_to_map/max_height", maxHeight_, 10.0) &&
                       nodeHandle_.param("/image_to_map/map_offset_x", xOffset_, 0.0) &&
                       nodeHandle_.param("/image_to_map/map_offset_y", yOffset_, 0.0) &&
                       nodeHandle_.getParam("save_path", savePath_);
        ROS_INFO_STREAM("ImageToMap: image_elevation_topic: " << imageElevationTopic_);
        ROS_INFO_STREAM("ImageToMap: image_occupancy_topic: " << imageOccupancyTopic_);
        ROS_INFO_STREAM("ImageToMap: resolution: " << resolution_);
        ROS_INFO_STREAM("ImageToMap: min_height: " << minHeight_);
        ROS_INFO_STREAM("ImageToMap: max_height: " << maxHeight_);
        ROS_INFO_STREAM("ImageToMap: save_path: " << savePath_);
        mapOrigin_ = Eigen::Vector2d(xOffset_, yOffset_);
        return success;
    }

    void ImageToMap::imageCallback(const sensor_msgs::ImageConstPtr &elevation_msgs,
                                   const sensor_msgs::ImageConstPtr &occupancy_msgs)
    {
        if (!mapInitialized_)
        {
            grid_map::GridMapRosConverter::initializeFromImage(*elevation_msgs, resolution_, map_, mapOrigin_);
            grid_map::GridMapRosConverter::addLayerFromImage(*elevation_msgs, "elevation", map_, 0., 10.);
            grid_map::GridMapRosConverter::addLayerFromImage(*occupancy_msgs, "occupancy", map_, minHeight_, maxHeight_);
            mapInitialized_ = true;
            // Debug 
            grid_map::Index mapIndex(0, 0);
            Eigen::Vector2d mapPosition;
            map_.getPosition(mapIndex, mapPosition);
            ROS_INFO_STREAM("ImageToMap: map origin: " << mapPosition);
            map_.setFrameId("map");
            grid_map::Index mapOriginIndex;
            Eigen::Vector2d mapOriginPosition(0, 0);
            map_.getIndex(mapOriginPosition, mapOriginIndex);
            ROS_INFO_STREAM("image index at origin of map frame " << mapOriginIndex);
            // grid_map::Index mapIndex2(112, 541);
            // Eigen::Vector2d mapPosition2;
            // map_.getPosition(mapIndex2, mapPosition2);
            // ROS_INFO_STREAM("image pos at origin " << mapPosition2);
        }
        publishMap();
    }

    void ImageToMap::publishMap()
    {
        grid_map_msgs::GridMap message;
        if (gridMapPublisher_.getNumSubscribers() > 0)
        {
            grid_map::GridMapRosConverter::toMessage(map_, message);
            gridMapPublisher_.publish(message);
        }
    }

    void ImageToMap::saveMap()
    {
        if (!savedMap_ && mapInitialized_)
        {
            ROS_INFO_STREAM("Saving map in " << savePath_);
            grid_map::GridMapRosConverter::saveToBag(map_, savePath_, std::string("grid_map"));
        }
        savedMap_ = true;
    }
}