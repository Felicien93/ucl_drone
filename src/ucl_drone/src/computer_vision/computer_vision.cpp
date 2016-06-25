/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/computer_vision/computer_vision.h>

ImageProcessor::ImageProcessor() : it_(nh_), pose_publishing(false), video_publishing(false)
{
  // Subscribe to input video
  bool autonomy_unavailable = false;
  bool flag_autonomy_unavailable = ros::param::get("~autonomy_unavailable", autonomy_unavailable);
  std::string drone_prefix = "/";
  bool flag_drone_prefix = ros::param::get("~drone_prefix", drone_prefix);
  std::string cam_type;  // bottom or front
  bool flag_cam_type = ros::param::get("~cam_type", cam_type);
  std::string video_channel;
  bool flag_video_channel = ros::param::get("~video_channel", video_channel);
  if (!flag_video_channel)
  {
    if (cam_type == "front")
    {
      video_channel = drone_prefix + "ardrone/front/image_raw";
    }
    else
    {
      cam_type = "bottom";
      video_channel = drone_prefix + "ardrone/bottom/image_raw";
    }
  }

  video_channel_ = nh_.resolveName(drone_prefix + video_channel);

  if (!autonomy_unavailable)
  {
    ros::ServiceClient client =
        nh_.serviceClient< ardrone_autonomy::CamSelect >(drone_prefix + "ardrone/setcamchannel");
    ardrone_autonomy::CamSelect srv;
    if (cam_type == "bottom")
    {
      srv.request.channel = 1;
    }
    else
    {
      srv.request.channel = 0;
    }
    int counter_call_success = 0;
    ros::Rate r(1 / 2.0);
    while (counter_call_success < 2)
    {
      ros::spinOnce();
      r.sleep();
      if (client.call(srv))
      {
        ROS_INFO("Camera toggled ?");
        if (srv.response.result)
          counter_call_success += 1;
      }
      else
      {
        ROS_INFO("Failed to call service setcamchannel, try it again in 1sec...");
      }
    }
  }

  image_sub_ =
      it_.subscribe(video_channel_, 1, &ImageProcessor::imageCb, this);  // second param: queue size

  pose_channel = nh_.resolveName("pose_estimation");
  pose_sub = nh_.subscribe(pose_channel, 1, &ImageProcessor::poseCb, this);

  processed_image_channel_out = nh_.resolveName("processed_image");
  processed_image_pub =
      nh_.advertise< ucl_drone::ProcessedImageMsg >(processed_image_channel_out, 1);

  target_loaded = target.init(TARGET_RELPATH);
}

ImageProcessor::~ImageProcessor()
{
}

/* This function is called every time a new image is published */
void ImageProcessor::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_DEBUG("ImageProcessor::imageCb");
  lastImageReceived = msg;
  video_publishing = true;
}

/* This function is called every time a new pose is published */
void ImageProcessor::poseCb(const ucl_drone::Pose3D::ConstPtr posePtr)
{
  ROS_DEBUG("ImageProcessor::poseCb");
  lastPoseReceived = *posePtr;
  pose_publishing = true;

  //! \todo ici compléter les images de la file des images qui attendent une pose
}

void ImageProcessor::publishProcessedImg()
{
  ROS_DEBUG("ImageProcessor::publishProcessedImg");
  ProcessedImage cam_img;
  cam_img.init(*lastImageReceived, lastPoseReceived);
  ucl_drone::ProcessedImageMsg::Ptr msg(new ucl_drone::ProcessedImageMsg);
  cam_img.convertToMsg(msg, target);
  processed_image_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "computer_vision");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  ImageProcessor ic;
  ROS_DEBUG("end of ImageProcessor initialization");

  ros::Rate r(12);  // 12Hz =  average frequency at which we receive images

  while ((!ic.pose_publishing || !ic.video_publishing) && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  while (ros::ok())
  {
    ros::spinOnce();  // if we dont want this we have to place callback and services in threads

    ic.publishProcessedImg();
    r.sleep();
  }

  return 0;
}
