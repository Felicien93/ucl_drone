/*!
 *  \file projection_2D.h
 *  \brief
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef UCL_DRONE_PROJECTION_2D_H
#define UCL_DRONE_PROJECTION_2D_H
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
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/ucl_drone.h>

void projection_2D(std::vector< cv::Point2f > &points_in, ucl_drone::Pose3D &pose,
                   std::vector< cv::Point3f > &points_out, bool h_flag = false);

#endif /* UCL_DRONE_PROJECTION_2D_H */
