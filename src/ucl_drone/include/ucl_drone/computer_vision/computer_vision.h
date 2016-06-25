/*!
 *  \file computer_vision.h
 *  \brief Basic image processor for the ardrone.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Image processor ROS node for the
 *  ardrone. Contains:
 *        - SIFT (from non-free opencv library) to detect keypoints
 *        - Image transformations
 *        - target detection
 */

#ifndef UCL_DRONE_COMPUTER_VISION_H
#define UCL_DRONE_COMPUTER_VISION_H
#define PCL_NO_PRECOMPILE

#include <boost/shared_ptr.hpp>

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

// vision
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

// messages
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>

// static const cv::SiftFeatureDetector detector;
// static const detector cv::FeatureDetector::create("SIFT");
// static const cv::SIFT detector(0, 3, 0.1, 15, 2);
// static const cv::SIFT detector(0, 3, 0.1, 20, 3);
static const cv::SIFT detector(0, 3, 0.1, 20, 3);  // limited to 300 features per frame
// static const cv::SurfFeatureDetector detector;
// static const cv::FastFeatureDetector detector;
// use FAST?
static const cv::SiftDescriptorExtractor extractor;

// ucl_drone

#include <ucl_drone/map/projection_2D.h>
#include <ucl_drone/ucl_drone.h>

#include <ucl_drone/computer_vision/target.h>

#include <ucl_drone/computer_vision/processed_image.h>

#include <ucl_drone/computer_vision/image_processor.h>

#endif /* UCL_DRONE_COMPUTER_VISION_H */
