/*!
 *  \file target_detection.h
 *  \brief Basic image processor for the ardrone.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Image processor ROS node for the
 *  ardrone. Contains:
 *        - SIFT (from non-free opencv library) to detect keypoints
 *        - Image transformations
 *        - Geometric transformations
 *        - Viewers
 */

#ifndef UCL_DRONE_TARGET_DETECTION_H
#define UCL_DRONE_TARGET_DETECTION_H

//! Filename to the target from within the package
static const std::string TARGET_RELPATH = "/target/target.jpg";

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

#include <signal.h>

// vision
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <ucl_drone/TargetDetected.h>

static const cv::SIFT detector(0, 3, 0.1, 20, 3);
// static const cv::SurfFeatureDetector detector;
// static const cv::FastFeatureDetector detector;
// use FAST?
static const cv::SiftDescriptorExtractor extractor;

/*!
 *  \class Target
 *  \brief Provide tools to track the presence of a target
 */
class Target
{
private:
  cv::Mat image;
  std::vector< cv::KeyPoint > keypoints;
  cv::Mat descriptors;
  std::vector< cv::Point2f > corners;
  std::vector< cv::Point2f > center;
  cv::FlannBasedMatcher matcher;

public:
  //! Constructor
  Target();

  //! Destructor
  ~Target();

  //! initializer
  bool init(const std::string relative_path);

  //! Detect the target
  bool detect(cv::Mat cam_img_descriptors, std::vector< cv::DMatch >& good_matches);

  //! draw a frame to indicate the detected target
  void draw(cv::Mat cam_img, std::vector< cv::KeyPoint > cam_keypoints,
            std::vector< cv::DMatch > good_matches, cv::Mat& img_matches);

  //! compute the position of the target on the camera image
  void position(std::vector< cv::KeyPoint > cam_keypoints, std::vector< cv::DMatch > good_matches,
                cv::Point2f& coord);
};

/*!
 *  \class TargetDetection
 *  \brief Node class
 */
class TargetDetection
{
private:
  // Publishers
  ros::Publisher target_pub;
  std::string target_channel_out;

  //! true if the target is successfully loaded
  bool target_loaded;

  Target target;

public:
  //! Constructor
  TargetDetection();

  //! Destructor
  ~TargetDetection();

  void targetDetectedPublisher();
}

#endif /* UCL_DRONE_TARGET_DETECTION_H */
