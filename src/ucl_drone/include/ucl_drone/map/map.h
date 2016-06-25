/*!
 *  \file map.h
 *  \brief Map wrapper
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef UCL_DRONE_MAP_H
#define UCL_DRONE_MAP_H
#define PCL_NO_PRECOMPILE

/* Header files */
#include <ros/package.h>
#include <ros/ros.h>

// ucl_drone
#include <ucl_drone/ucl_drone.h>

/* Point Cloud library */
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>  // pcl::PointXYZRGB
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>              // pcl::PointCloud
#include <ucl_drone/PointXYZRGBSIFT.h>

/* Boost */
#include <boost/thread/thread.hpp>

/* Messages */
#include <sensor_msgs/PointCloud2.h>  // sensor_msgs::PointCloud2

/* ucl_drone */
#include <ucl_drone/FeaturesArray.h>
#include <ucl_drone/MapChunk.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/Projection.h>

/** \class Map
 *  A description here
 */
class Map
{
private:
  /* Attributes */

  ros::NodeHandle nh;

  //! The cloud object containing 3D points with a descriptor (cv::Mat)
  pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr cloud;

  //! The visualizer object to to perform projection
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer;

  /* Subscribers */

  //! \see{Map::poseCb}
  ros::Subscriber pose_sub;
  std::string pose_channel;

  //! Callback when pose is received, update the drone pose in the map \todo{not implemented}
  void poseCb(const ucl_drone::Pose3D::ConstPtr pose);

  /* Services Definition */

  //! \see{Map::requestProjectionHandler}
  ros::ServiceServer requestProjection;

  //! Provide the projected point cloud on the view of the camera (\todo{config file}) from the
  //! input pose3D
  //! \todo{not implemented}
  bool requestProjectionHandler(ucl_drone::Projection::Request &,
                                ucl_drone::Projection::Response &);

  //! \see{Map::processFeaturesHandler}
  ros::ServiceServer processFeatures;

  //! Add new points or re-estimate their position \todo{not implemented}
  bool processFeaturesHandler(ucl_drone::FeaturesArray::Request &,
                              ucl_drone::FeaturesArray::Response &);

  //! \see{Map::mergeMapsHandler}
  ros::ServiceServer mergeMaps;

  //! search intersection between two (parts of two) point clouds, merge \todo{not implemented}
  bool mergeMapsHandler(ucl_drone::MapChunk::Request &, ucl_drone::MapChunk::Response &);

public:
  //! Contructor. Initialize an empty map
  Map();

  //! Destructor.
  ~Map();

  // Define here functions you need to call from other object classes inside the same node
};

#endif /* UCL_DRONE_MAP_H */
