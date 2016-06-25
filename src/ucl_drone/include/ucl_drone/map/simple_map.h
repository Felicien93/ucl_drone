/*!
 *  \file simple_map.h
 *  \brief Simple map to visualize simple 3D pointcloud
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef UCL_DRONE_SIMPLE_MAP_H
#define UCL_DRONE_SIMPLE_MAP_H
#define PCL_NO_PRECOMPILE

/* Header files */
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* Point Cloud library */
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>  // pcl::PointXYZRGB
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>              // pcl::PointCloud

/* Boost */
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

/* Messages */
//#include <sensor_msgs/PointCloud2.h>  // sensor_msgs::PointCloud2
#include <sensor_msgs/image_encodings.h>

/* ucl_drone */
#include <ucl_drone/PointXYZRGBSIFT.h>  // pcl::PointXYZRGBSIFT
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>
#include <ucl_drone/TargetDetected.h>
#include <ucl_drone/map/frame.h>
#include <ucl_drone/map/projection_2D.h>
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/ucl_drone.h>

/** \class Map
 *  A description here
 */
class Map
{
private:
  /* Attributes */

  ros::NodeHandle nh;

  bool processedImgReceived;
  //! The viewer
  // pcl::visualization::CloudViewer viewer;

  /* Subscribers */
  //! Processed_image subscriber
  ros::Subscriber processed_image_sub;
  std::string processed_image_channel_in;

  /* Publishers */
  ros::Publisher pose_PnP_pub;
  std::string pose_PnP_channel;

  ros::Publisher target_pub;
  std::string target_channel_out;

  //! Callback when image is received
  void processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in);

  // Measure
  ucl_drone::ProcessedImageMsg::ConstPtr lastProcessedImgReceived;

  // void comparePointsWithMap(const pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr
  // pointcloud_in,
  //                           pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud_unknown);

  /* Services Definition */

  // boost::mutex cloud_mutex;
  cv::Mat camera_matrix_K;

  cv::Mat tvec;
  cv::Mat rvec;

public:
  //! The cloud object containing 3D points
  pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr cloud;

  //! The visualizer object to perform projection
  boost::shared_ptr< pcl::visualization::PCLVisualizer > visualizer;

  //! Contructor. Initialize an empty map
  Map();

  //! Destructor.
  ~Map();

  void getDescriptors(ucl_drone::Pose3D pose, cv::Mat &descriptors, std::vector< int > &idx);
  // Define here functions you need to call from other object classes inside the same node

  void targetDetectedPublisher();

  void visu_loop();
};

#endif /* UCL_DRONE_SIMPLE_MAP_H */
