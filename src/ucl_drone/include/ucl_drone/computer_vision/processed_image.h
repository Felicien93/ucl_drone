

#ifndef UCL_DRONE_PROCESSED_IMAGE_H
#define UCL_DRONE_PROCESSED_IMAGE_H

#include <ucl_drone/computer_vision/computer_vision.h>

/*!
 *  \class ProcessedImage
 *  \brief Provide tools to manipulate an camera image
 */
class ProcessedImage
{
private:
  bool has_pose_;
  ucl_drone::ProcessedImageMsg::Ptr msg;

public:
  cv_bridge::CvImagePtr cv_ptr;
  std::vector< cv::KeyPoint > keypoints;
  cv::Mat descriptors;
  sensor_msgs::Image image;

  ucl_drone::Pose3D pose;
  ros::Time stamp;

  ProcessedImage();
  ~ProcessedImage();

  void init(const sensor_msgs::Image image, const ucl_drone::Pose3D pose);

  void drawKeypoints(cv::Mat& output);

  bool addPose(const ucl_drone::Pose3D pose);
  bool hasPose()
  {
    return has_pose_;
  };
  void convertToMsg(ucl_drone::ProcessedImageMsg::Ptr& msg, Target target);
};

#endif /*UCL_DRONE_PROCESSED_IMAGE_H*/
