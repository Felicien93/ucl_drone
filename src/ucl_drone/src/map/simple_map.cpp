/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/map/simple_map.h>

void my_hard_pointcloud(pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr cloud)
{
  // Fill in the cloud data
  cloud->width = 5;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    int rgb_ = 255 << 16 | 0 << 8 | 0;
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].rgb = *reinterpret_cast< float* >(&rgb_);
  }

  // pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr ptrCloud(&cloud);
  // pcl::PointCloud< pcl::PointXYZ >::ConstPtr ptrCloud = &cloud;

  // return cptrCloud;
}

void my_hard_pointcloud2(pcl::PointCloud< pcl::PointXYZRGBSIFT >& cloud)
{
  // Fill in the cloud data
  int rgb_ = 255 << 16 | 0 << 8 | 0;
  cloud.points[0].x = 0;
  cloud.points[1].x = 0;
  cloud.points[2].x = 0;

  cloud.points[0].z = 1.1;
  cloud.points[1].z = 1.4;
  cloud.points[2].z = 1.7;

  cloud.points[0].y = 0;
  cloud.points[1].y = 0;
  cloud.points[2].y = 0;

  cloud.points[0].rgb = *reinterpret_cast< float* >(&rgb_);
  cloud.points[1].rgb = *reinterpret_cast< float* >(&rgb_);
  cloud.points[2].rgb = *reinterpret_cast< float* >(&rgb_);
}

void cloud_debug(pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr cloud)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    ROS_DEBUG("points[%d] = (%f, %f, %f)", i, cloud->points[i].x, cloud->points[i].y,
              cloud->points[i].z);
  }
}

Map::Map()
  : visualizer(new pcl::visualization::PCLVisualizer("3D visualizer"))
  , cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
{
  this->processedImgReceived = false;
  // Subsribers
  processed_image_channel_in = nh.resolveName("processed_image");
  processed_image_sub =
      nh.subscribe(processed_image_channel_in, 10, &Map::processedImageCb,
                   this);  // carefull!!! size of queue is 10 to permit to manipulate the viewer
                           // (which blocks the rest of program execution) without losing too much
                           // data. Be carefull to have sufficient debit so it makes sense .

  // Publishers
  target_channel_out = nh.resolveName("ucl_drone/target_detected");
  target_pub = nh.advertise< ucl_drone::TargetDetected >(target_channel_out, 1);

  //  boost::shared_ptr< pcl::visualization::PCLVisualizer > visualizer(
  //      new pcl::visualization::PCLVisualizer("3D viewer"));

  // my_hard_pointcloud(cloud);

  // pcl::visualization::PointCloudColorHandlerRGBField< pcl::PointXYZRGBSIFT > rgb(cloud);
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointXYZRGBSIFT > single_color(cloud, 255,
                                                                                        0, 0);
  visualizer->setBackgroundColor(0, 0.1, 0.3);
  // visualizer->addPointCloud< pcl::PointXYZRGBSIFT >(cloud, rgb, "SIFT_cloud");
  visualizer->addPointCloud< pcl::PointXYZRGBSIFT >(cloud, single_color, "SIFT_cloud");
  visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                               "SIFT_cloud");
  visualizer->addCoordinateSystem(1.0);
  visualizer->initCameraParameters();
  // pcl::visualization::Camera cam;
  // visualizer->getCameraParameters(cam);
  // cam.pos[3] = 10;
  // cam.view[0] = 1;
  // cam.view[1] = 0;
  // visualizer->setCameraParameters(cam);

  this->camera_matrix_K = (cv::Mat_< double >(3, 3) << FOCAL_LENGTH_x, 0, IMG_CENTER_x, 0,
                           FOCAL_LENGTH_y, IMG_CENTER_y, 0, 0, 1);

  ROS_DEBUG("simple_map initialized");

  pose_PnP_channel = nh.resolveName("pnp");
  pose_PnP_pub = nh.advertise< ucl_drone::Pose3D >(pose_PnP_channel, 1);

  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);
}

Map::~Map()
{
}

void Map::processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in)
{
  // ROS_DEBUG("Map::processedImageCb start");
  this->processedImgReceived = true;
  this->lastProcessedImgReceived = processed_image_in;
  Frame current_frame(processed_image_in);

  std::vector< std::pair< int, int > > idx_matching_points;
  std::vector< int > idx_unknown_points;
  std::vector< cv::Point3f > map_matching_points;
  std::vector< cv::Point2f > frame_matching_points;
  std::vector< cv::Point2f > frame_unknown_points;

  current_frame.matchWithMap(*this, idx_matching_points, idx_unknown_points, map_matching_points,
                             frame_matching_points, frame_unknown_points);

  // if (frame_unknown_points.size() == 0)
  // {
  //   ROS_DEBUG("NOTHING TO ADD !");
  //   ROS_DEBUG("# img:%4d, map:%4d, match:%4d, inliers::%4d", current_frame.imgPoints.size(),
  //             this->cloud->points.size(), map_matching_points.size(), 0);
  //   return;
  // }
  // else
  // {
  //   // ROS_DEBUG("%d points to add !", frame_unknown_points.size());
  // }

  ucl_drone::Pose3D mixed_pose;
  std::vector< cv::Point3f > points_out;  // points after 2D projection
  if (map_matching_points.size() > 7)
  {
    cv::Mat_< double > tcam, cam2world, world2drone,
        distCoeffs;  // give the distcoeff after re-calibration?
    // cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    // cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    std::vector< int > inliers;
    // distCoeffs = (cv::Mat_< double >(1, 5) << -0.511605, 0.270084, 0.000568, 0.000973, 0.000000);

    distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);

    // cv::Mat opoints = map_matching_points.getMat(), ipoints = frame_matching_points.getMat();
    // int npoints = std::max(map_matching_points.checkVector(3, CV_32F),
    //                        map_matching_points.checkVector(3, CV_64F));
    // ROS_DEBUG("%d %d", npoints >= 0,
    //           npoints == std::max(frame_matching_points.checkVector(2, CV_32F),
    //                               frame_matching_points.checkVector(2, CV_64F)));
    cv::solvePnPRansac(map_matching_points, frame_matching_points, this->camera_matrix_K,
                       distCoeffs, rvec, tvec, true, 3000, 2, 500, inliers,
                       CV_P3P);  // CV_ITERATIVE);  // CV_EPNP);

    ROS_DEBUG("# img:%4d, map:%4d, match:%4d, inliers::%4d", current_frame.imgPoints.size(),
              this->cloud->points.size(), map_matching_points.size(), inliers.size());

    // ROS_DEBUG("rvec: %+2.6f, %+2.6f, %+2.6f", rvec.at< double >(0, 0), rvec.at< double >(1, 0),
    //           rvec.at< double >(2, 0));

    if (inliers.size() < 7 || inliers.size() < map_matching_points.size() / 7.0)
    {
      ROS_DEBUG("TRACKING LOST ! (not enough inliers)");
      return;
    }

    if (cv::norm(tvec) > 100.0)
    {
      ROS_DEBUG("TRACKING LOST ! (norm of translation)");
      return;
    }

    cv::Rodrigues(rvec, cam2world);

    if (fabs(determinant(cam2world)) - 1 > 1e-07)
    {
      ROS_DEBUG("TRACKING LOST ! (determinant of rotation matrix)");
      return;
    }

    cv::Mat drone2cam = rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
    // cout << "drone2cam = " << endl << " " << drone2cam << endl << endl;
    cv::Mat_< double > drone2cambis =
        (cv::Mat_< double >(3, 3) << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0,
         -1.0);  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! bizarre, sur papier different
    // cout << "drone2cambis = " << endl << " " << drone2cambis << endl << endl;

    tcam = -cam2world.t() * tvec;
    ucl_drone::Pose3D PnP_pose;
    PnP_pose.x = tcam(0);
    PnP_pose.y = tcam(1);
    PnP_pose.z = tcam(2);

    cv::Mat_< double > world2cam = cam2world.t();
    cv::Mat_< double > cam2drone = drone2cambis.t();
    world2drone = world2cam * cam2drone;  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  bizarre, sur
                                          // papier different

    tf::Matrix3x3(world2drone(0, 0), world2drone(0, 1), world2drone(0, 2), world2drone(1, 0),
                  world2drone(1, 1), world2drone(1, 2), world2drone(2, 0), world2drone(2, 1),
                  world2drone(2, 2))
        .getRPY(PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);

    // tcam = -cam2world.t() * tvec;
    // ucl_drone::Pose3D PnP_pose;
    // PnP_pose.x = tcam(0);
    // PnP_pose.y = tcam(1);
    // PnP_pose.z = tcam(2);
    //
    // world2drone = drone2cam.t() * cam2world.t();

    // tf::Matrix3x3(world2drone(0, 0), world2drone(0, 1), world2drone(0, 2), world2drone(1, 0),
    //               world2drone(1, 1), world2drone(1, 2), world2drone(2, 0), world2drone(2, 1),
    //               world2drone(2, 2))
    //     .getRPY(PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);

    // tf::Matrix3x3(world2drone(0, 0), world2drone(1, 0), world2drone(2, 0), world2drone(0, 1),
    //               world2drone(1, 1), world2drone(2, 1), world2drone(0, 2), world2drone(1, 2),
    //               world2drone(2, 2))
    //     .getRPY(PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);

    // The following code is for Euler-YPR conversion, not RPY ! see
    // http://planning.cs.uiuc.edu/node103.html
    // if (world2drone(2, 2) == 0 || world2drone(0, 0) == 0)
    // {
    //   ROS_ERROR("Bad transformation matrix !");
    // }
    // PnP_pose.rotX = atan2(world2drone(2, 1), world2drone(2, 2));
    // PnP_pose.rotY = atan2(-world2drone(2, 0), sqrt(world2drone(2, 1) * world2drone(2, 1) +
    //                                                world2drone(2, 2) * world2drone(2, 2)));
    // PnP_pose.rotZ = atan2(world2drone(1, 0), world2drone(0, 0));

    PnP_pose.xvel = 0.0;
    PnP_pose.yvel = 0.0;
    PnP_pose.zvel = 0.0;
    PnP_pose.rotXvel = 0.0;
    PnP_pose.rotYvel = 0.0;
    PnP_pose.rotZvel = 0.0;

    PnP_pose.header.stamp = processed_image_in->pose.header.stamp;  // needed for rqt_plot
    pose_PnP_pub.publish(PnP_pose);

    ROS_DEBUG("PnP_pose:  x=%+2.6f y=%+2.6f z=%+2.6f rotX=%+2.6f rotY=%+2.6f rotZ=%+2.6f",
              PnP_pose.x, PnP_pose.y, PnP_pose.z, PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);
    ROS_DEBUG("trad_pose: x=%+2.6f y=%+2.6f z=%+2.6f rotX=%+2.6f rotY=%+2.6f rotZ=%+2.6f",
              current_frame.msg.pose.x, current_frame.msg.pose.y, current_frame.msg.pose.z,
              current_frame.msg.pose.rotX, current_frame.msg.pose.rotY,
              current_frame.msg.pose.rotZ);

    mixed_pose = current_frame.msg.pose;
    mixed_pose.x = PnP_pose.x;
    mixed_pose.y = PnP_pose.y;
    mixed_pose.z = PnP_pose.z;
    mixed_pose.rotZ = PnP_pose.rotZ;

    if (frame_unknown_points.size() == 0)
    {
      ROS_DEBUG("NOTHING TO ADD !");
      return;  // do note projection with no points
    }
    projection_2D(frame_unknown_points, mixed_pose, points_out);

    // if (current_frame.imgPoints.size() > 2 * map_matching_points.size())
    // {
    pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud(
        new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
    current_frame.convertToPcl(idx_unknown_points, points_out, pointcloud);
    //*(this->cloud) = *pointcloud;
    //*(this->cloud) += *pointcloud;

    this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
    // }
  }
  else
  {
    ROS_DEBUG("TRACKING LOST ! (not enough matching points)");

    // projection_2D(current_frame.imgPoints, current_frame.msg.pose, points_out, true);
    projection_2D(frame_unknown_points, current_frame.msg.pose, points_out, true);
    // projection_2D(frame_unknown_points, pose, points_out);
    pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud(
        new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
    current_frame.convertToPcl(idx_unknown_points, points_out, pointcloud);
    *(this->cloud) = *pointcloud;
    this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
  }

  // my_hard_pointcloud2(*(this->cloud));
  // this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
}
// {
//   // ROS_DEBUG("%f,%f,%f", pointcloud_in->points[0].x, pointcloud_in->points[0].y,
//   // pointcloud_in->points[0].z);
//
//   // pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >);
//   // equivalent to:
//   // boost::shared_ptr< pcl::PointCloud< pcl::pcl::PointXYZRGBSIFT > > cloud(new
//   // pcl::PointCloud< pcl::PointXYZRGBSIFT >);
//
//   // my_hard_pointcloud(cloud);
//   // viewer.showCloud(cloud);
//
//   // viewer.showCloud(pointcloud_in);  //, "test");
//
//   pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud_unknown(
//       new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
//   comparePointsWithMap(pointcloud_in, pointcloud_unknown);
//   ROS_DEBUG("simple_map before pointcloud update");
//   *cloud += *pointcloud_unknown;
//
//   // ROS_DEBUG("POINTCLOUD_IN SIZE: %d", pointcloud_in->points.size());
//   // cloud_debug(pointcloud_in);
//   // //*cloud += *pointcloud_in;
//   // // *cloud = *pointcloud_in;
//   visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(cloud, "SIFT_cloud");
// }

//! TODO: for the moment, detection with 2D hypothesis. In the future, use PnP
void Map::targetDetectedPublisher()
{
  if (processedImgReceived && target_pub.getNumSubscribers() > 0)
  {
    ROS_DEBUG("signal sent ImageProcessor::targetDetectedPublisher");

    if (lastProcessedImgReceived->target_detected)
    {
      ROS_DEBUG("TARGET IS DETECTED");

      ucl_drone::TargetDetected msg;
      msg.pose = lastProcessedImgReceived->pose;
      // msg.navdata = lastProcessedImgReceived->navdata;
      msg.img_point.x = lastProcessedImgReceived->target_points[4].x;
      msg.img_point.y = lastProcessedImgReceived->target_points[4].y;
      msg.img_point.z = 0;
      std::vector< cv::Point2f > target_center(1);
      target_center[0].x = lastProcessedImgReceived->target_points[4].x;
      target_center[0].y = lastProcessedImgReceived->target_points[4].y;
      std::vector< cv::Point3f > world_coord;
      projection_2D(target_center, msg.pose, world_coord);
      msg.world_point.x = world_coord[0].x;
      msg.world_point.y = world_coord[0].y;
      msg.world_point.z = world_coord[0].z;
      target_pub.publish(msg);
    }
  }
}

//! return the entire map
void Map::getDescriptors(ucl_drone::Pose3D pose, cv::Mat& descriptors, std::vector< int >& idx)
{
  descriptors = cv::Mat_< float >(cloud->points.size(), 128);
  for (unsigned i = 0; i < cloud->points.size(); ++i)
  {
    int rgb_ = 255 << 16 | 0 << 8 | 0;
    cloud->points[i].x;
    cloud->points[i].y;
    cloud->points[i].z;
    for (unsigned j = 0; j < 128; ++j)
    {
      descriptors.at< float >(i, j) = cloud->points[i].descriptor[j];
    }
    idx.push_back(i);
  }
}

// void Map::comparePointsWithMap(const pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr
// pointcloud_in,
//                                pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud_unknown)
// {
//   pointcloud_unknown->is_dense = false;
//
//   cv::Mat map_descriptors;
//   getDescriptors(cloud, map_descriptors);
//   cv::Mat in_descriptors;
//   getDescriptors(pointcloud_in, in_descriptors);
//
//   cv::FlannBasedMatcher matcher;
//   std::vector< cv::DMatch > simple_matches;
//   std::vector< int > idx;
//   if (in_descriptors.rows == 0)
//   {
//     return;
//   }
//   if (map_descriptors.rows != 0)
//   {
//     matcher.match(in_descriptors, map_descriptors, simple_matches);
//   }
//   // threashold test
//   for (unsigned k = 0; k < simple_matches.size(); k++)
//   {
//     if (simple_matches[k].distance < 200.0)
//     {
//       idx.push_back(simple_matches[k].queryIdx);
//     }
//   }
//   unsigned j = 0;
//   ROS_DEBUG("idx.size() %d", idx.size());
//   if (idx.size() == 0)
//   {
//     idx.push_back(pointcloud_in->points.size() + 1);
//   }
//   for (unsigned k = 0; k < pointcloud_in->points.size(); k++)
//   {
//     if (idx[j] == k)
//     {
//       j++;
//     }
//     else
//     {
//       pointcloud_unknown->points.push_back(pointcloud_in->points[k]);
//     }
//   }
//   ROS_DEBUG("pointcloud_unknown->points.size() %d", pointcloud_unknown->points.size());
// }

// void Map::visu_loop()
// {
//   ROS_DEBUG("Map:  visu_loop");
//   ros::Rate r(8);
//   while (ros::ok())
//   {
//     this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
//     ROS_DEBUG("Map:  visualizer->spinOnce");
//     this->visualizer->spinOnce(100);
//     r.sleep();
//     boost::this_thread::interruption_point();
//   }
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_map");
  ROS_INFO_STREAM("simple map started!");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  Map map;

  ros::Rate r(12);

  int visualizer_count = 0;

  // boost::thread visu_thread(&Map::visu_loop, &map);

  while (ros::ok())
  {
    // To reduce rate of visualizer by 5
    // if (visualizer_count >= 5)
    // {
    //   ROS_DEBUG("Map:  visualizer->spinOnce");
    map.visualizer->spinOnce(100);
    //   visualizer_count = 0;
    // }
    // visualizer_count += 1;

    map.targetDetectedPublisher();

    // pcl::visualization::Camera cam;
    // // Save the position of the camera
    // map.visualizer->getCameraParameterscam);
    // // Print recorded points on the screen:
    // cout << "Cam: " << endl
    //      << " - pos: (" << cam.pos[0] << ", " << cam.pos[1] << ", " << cam.pos[2] << ")" << endl
    //      << " - view: (" << cam.view[0] << ", " << cam.view[1] << ", " << cam.view[2] << ")" <<
    //      endl
    //      << " - focal: (" << cam.focal[0] << ", " << cam.focal[1] << ", " << cam.focal[2] << ")"
    //      << endl;

    ros::spinOnce();  // if we dont want this we have to place callback and services in threads
    r.sleep();
  }
  // visu_thread.interrupt();
  return 0;
}
