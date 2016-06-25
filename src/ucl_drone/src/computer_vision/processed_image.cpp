/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  based on tuto:
 *  http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 */

#include <ucl_drone/computer_vision/computer_vision.h>

ProcessedImage::ProcessedImage() : has_pose_(false)
{
  ucl_drone::ProcessedImageMsg::Ptr msg(new ucl_drone::ProcessedImageMsg());
}

void ProcessedImage::init(const sensor_msgs::Image msg, const ucl_drone::Pose3D pose_)
{
  ROS_DEBUG("ProcessedImage::init");
  this->pose = pose_;
  // this->stamp = msg.header.stamp;
  // this->image = msg;

  // convert ROS image to OpenCV image
  try
  {
    this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // ROS_DEBUG("ProcessedImage::init: the size of the corrected image is: %d, %d",
    // cv_ptr->image.rows, cv_ptr->image.cols);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("ucl_drone::imgproc::cv_bridge exception: %s", e.what());
    return;
  }

  cv::Size size(735, 360);
  cv::resize(this->cv_ptr->image, this->cv_ptr->image, size);

  this->cv_ptr->toImageMsg(this->image);

  // detect and fill keypoints vector for current image
  detector.detect(this->cv_ptr->image, this->keypoints);
  ROS_DEBUG("ProcessedImage::init this->keypoints.size()=%d", this->keypoints.size());
  extractor.compute(this->cv_ptr->image, this->keypoints, this->descriptors);

  ROS_DEBUG("end ProcessedImage::init");
}

ProcessedImage::~ProcessedImage()
{
}

void ProcessedImage::convertToMsg(ucl_drone::ProcessedImageMsg::Ptr& msg, Target target)
{
  msg->pose = this->pose;
  msg->image = this->image;
  msg->keypoints.resize(this->keypoints.size());

  for (unsigned i = 0; i < this->keypoints.size(); i++)
  {
    ucl_drone::KeyPoint keypoint;

    geometry_msgs::Point point;
    point.x = (double)this->keypoints[i].pt.x;
    point.y = (double)this->keypoints[i].pt.y;

    keypoint.point = point;

    std::vector< float > descriptor;
    descriptor.resize(128);
    for (unsigned k = 0; k < 128; k++)  // 128 is the size of a SIFT descriptor
    {
      descriptor[k] = this->descriptors.at< float >(i, k);
    }
    keypoint.descriptor = descriptor;

    msg->keypoints[i] = keypoint;

    // ROS_DEBUG("POINT[%d] (%f,%f) vs (%f,%f)", i, this->keypoints[i].pt.x,
    // this->keypoints[i].pt.y,
    //          msg->keypoints[i].point.x, msg->keypoints[i].point.y);
  }

  std::vector< cv::DMatch > good_matches;
  bool target_is_detected = target.detect(this->descriptors, good_matches
#ifdef DEBUG_PROJECTION
                                          ,
                                          this->keypoints, this->pose
#endif
                                          );
  if (target_is_detected)
  {
    ROS_DEBUG("TARGET IS DETECTED");

    std::vector< cv::Point2f > target_coord;
    target.position(this->keypoints, good_matches, target_coord);
    msg->target_detected = true;
    msg->target_points.resize(5);
    for (int i = 0; i < 5; i++)
    {
      msg->target_points[i].x = target_coord[i].x;
      msg->target_points[i].y = target_coord[i].y;
      msg->target_points[i].z = 0;
    }
  }
  else
  {
    msg->target_detected = false;
  }
}

// void ProcessedImage::drawKeypoints(cv::Mat& output)
// {
//   cv::drawKeypoints(cv_ptr->image, keypoints, output);  //, cv::DrawMatchesFlags::DEFAULT);
//   ROS_DEBUG("end ProcessedImage::drawKeypoints");
// }

bool ProcessedImage::addPose(const ucl_drone::Pose3D pose_)
{
  if (has_pose_)
  {
    return false;
  }
  pose = pose_;
  has_pose_ = true;
  return true;
}
