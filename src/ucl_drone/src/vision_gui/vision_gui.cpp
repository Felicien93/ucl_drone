/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/vision_gui/vision_gui.h>

VisionGui::VisionGui()
{
  // Instantiate the two viewer widows
  cv::namedWindow(OPENCV_WINDOW1);

  // Subscribe to processed_image (image + keypoints + ...)
  processed_img_channel = nh_.resolveName("processed_image");
  processed_img_sub = nh_.subscribe(processed_img_channel, 1, &VisionGui::processedImageCb, this);

  new_processed_img_available = false;
  target_detected = false;
}

VisionGui::~VisionGui()
{
  cv::destroyWindow(OPENCV_WINDOW1);
}

/* This function is called every time a new processed_image is published */
void VisionGui::processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image)
{
  ROS_DEBUG("VisionGui::processedImageCb start");
  this->lastProcessedImgReceived = processed_image;
  convertMsgToAttributes(processed_image);
  // processed_image_publishing = true;
  new_processed_img_available = true;
}

void VisionGui::guiDrawKeypoints()
{
  ROS_DEBUG("signal sent ImageProcessor::guiDrawKeypoints");

  // Show all keypoints in image
  cv::Mat output = cv_ptr->image;
  for (unsigned i = 0; i < keypoints.size(); ++i)
  {
    cv::circle(output, keypoints[i], 3, cv::Scalar(0, 0, 255), -1, 8);
  }

  // Show the target
  if (target_detected)
  {
    cv::line(output, this->target_cornersAndCenter[0], this->target_cornersAndCenter[1],
             cv::Scalar(0, 255, 0), 4);
    cv::line(output, this->target_cornersAndCenter[1], this->target_cornersAndCenter[2],
             cv::Scalar(0, 255, 0), 4);
    cv::line(output, this->target_cornersAndCenter[2], this->target_cornersAndCenter[3],
             cv::Scalar(0, 255, 0), 4);
    cv::line(output, this->target_cornersAndCenter[3], this->target_cornersAndCenter[0],
             cv::Scalar(0, 255, 0), 4);
  }

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW1, output);
  cv::waitKey(3);
}

void VisionGui::convertMsgToAttributes(ucl_drone::ProcessedImageMsg::ConstPtr msg)
{
  // convert msg to opencv format
  this->keypoints.resize(msg->keypoints.size());
  this->descriptors = cv::Mat_< float >(msg->keypoints.size(), 128);
  for (unsigned i = 0; i < msg->keypoints.size(); ++i)
  {
    //    C++: KeyPoint::KeyPoint(Point2f _pt, float _size, float _angle=-1, float _response=0, int
    //    _octave=0, int _class_id=-1)
    // this->keypoints[i] = cv::KeyPoint(msg->keypoints[i], (float)0, (float)-1, (float)0, 0, -1);

    this->keypoints[i].x = (double)msg->keypoints[i].point.x;
    this->keypoints[i].y = (double)msg->keypoints[i].point.y;

    // ROS_DEBUG("POINT[%d] (%f,%f) vs (%f,%f)", i, this->imgPoints[i].x, this->imgPoints[i].y,
    //           msg->keypoints[i].point.x, msg->keypoints[i].point.y);

    for (unsigned j = 0; j < 128; ++j)
    {
      this->descriptors.at< float >(i, j) = (float)msg->keypoints[i].descriptor[j];
    }
  }
  target_detected = msg->target_detected;
  if (target_detected)
  {
    this->target_cornersAndCenter.resize(msg->target_points.size());
    for (unsigned i = 0; i < msg->target_points.size(); ++i)
    {
      this->target_cornersAndCenter[i].x = msg->target_points[i].x;
      this->target_cornersAndCenter[i].y = msg->target_points[i].y;
    }
  }

  // convert ROS image to OpenCV image
  try
  {
    this->cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("ucl_drone::VisionGui::cv_bridge exception: %s", e.what());
    return;
  }
}

// void VisionGui::guiDrawTarget(Synchro* synchro)
// {
//   ros::Rate r(2);
//   while (ros::ok())
//   {
//     if (target_loaded)
//     {
//       synchro->sendRequest();
//       ROS_DEBUG("signal sent ImageProcessor::guiDrawTarget");
//
//       boost::shared_ptr< ProcessedImage > cam_img = synchro->getResponse();
//
//       std::vector< cv::DMatch > good_matches;
//       bool target_is_detected = target.detect(cam_img->descriptors, good_matches);
//       if (target_is_detected)
//       {
//         cv::Mat img_matches;
//         target.draw(cam_img->cv_ptr->image, cam_img->keypoints, good_matches, img_matches);
//         cv::imshow(OPENCV_WINDOW2, img_matches);
//         cv::waitKey(3);
//       }
//     }
//     r.sleep();
//     boost::this_thread::interruption_point();
//   }
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_gui");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  VisionGui my_gui;
  ROS_DEBUG("end of computer vision viewer initialization");

  ros::Rate r(12);

  while (ros::ok())
  {
    ros::spinOnce();
    if (my_gui.new_processed_img_available)
    {
      my_gui.guiDrawKeypoints();
      my_gui.new_processed_img_available = false;
    }
    r.sleep();
  }

  return 0;
}
