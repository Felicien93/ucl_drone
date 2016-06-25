/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

TargetDetection::TargetDetection()
{
  target_channel_out = nh_.resolveName("ucl_drone/target_detected");
  target_pub = nh_.advertise< ucl_drone::TargetDetected >(target_channel_out, 1);

  target_loaded = target.init(TARGET_RELPATH);
}

// il manque un image processed callback

// à mettre à jour en fonction du contenu du message
void TargetDetection::targetDetectedPublisher()
{
  if (target_pub.getNumSubscribers() > 0)
  {
    if (target_loaded)
    {
      synchro->sendRequest();
      ROS_DEBUG("signal sent ImageProcessor::targetDetectedPublisher");

      std::vector< cv::DMatch > good_matches;
      bool target_is_detected = target.detect(cam_img->descriptors, good_matches);
      if (target_is_detected)
      {
        ROS_DEBUG("TARGET IS DETECTED");

        cv::Point2f img_coord;
        target.position(cam_img->keypoints, good_matches, img_coord);
        ucl_drone::TargetDetected msg;
        msg.pose = cam_img->pose;
        msg.navdata = cam_img->navdata;
        msg.img_point.x = img_coord.x;
        msg.img_point.y = img_coord.y;
        msg.img_point.z = 0;
        cv::Point3d world_coord;
        convert_point_to_mappoint(img_coord, cam_img->pose, world_coord);
        msg.world_point.x = world_coord.x;
        msg.world_point.y = world_coord.y;
        msg.world_point.z = world_coord.z;
        target_pub.publish(msg);
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_detection");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  TargetDetection target_detector();
  ROS_DEBUG("Target detection launched");
  ros::Rate r(4);
  while (ros::ok())
  {
    target_detector.targetDetectedPublisher();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
