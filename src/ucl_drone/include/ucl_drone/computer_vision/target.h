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

#ifndef UCL_DRONE_TARGET_H
#define UCL_DRONE_TARGET_H

//#define DEBUG_PROJECTION  // if defined print relative errors of projection for the target

#include <ucl_drone/computer_vision/computer_vision.h>

//! Filename to the target from within the package
static const std::string TARGET_RELPATH = "/target/target.jpg";

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
  std::vector< cv::Point2f > centerAndCorners;
  cv::FlannBasedMatcher matcher;

public:
  //! Constructor
  Target();

  //! Destructor
  ~Target();

  //! initializer
  bool init(const std::string relative_path);

  //! Detect the target
  bool detect(cv::Mat cam_img_descriptors, std::vector< cv::DMatch >& good_matches
#ifdef DEBUG_PROJECTION
              ,
              std::vector< cv::KeyPoint >& cam_keypoints, ucl_drone::Pose3D& pose
#endif
              );

  //! draw a frame to indicate the detected target
  void draw(cv::Mat cam_img, std::vector< cv::KeyPoint > cam_keypoints,
            std::vector< cv::DMatch > good_matches, cv::Mat& img_matches);

  //! compute the position of the target on the camera image
  void position(std::vector< cv::KeyPoint > cam_keypoints, std::vector< cv::DMatch > good_matches,
                std::vector< cv::Point2f >& coord);
};

#endif /* UCL_DRONE_TARGET_DETECTION_H */
