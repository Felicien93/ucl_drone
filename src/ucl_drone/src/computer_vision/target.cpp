/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  tuto:
 * http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
 */

#include <ucl_drone/computer_vision/computer_vision.h>

//! Absolute path to the package
static const std::string PKG_DIR = ros::package::getPath("ucl_drone");

Target::Target()
{
}

Target::~Target()
{
}

bool Target::init(const std::string relative_path)
{
  std::string str = PKG_DIR + relative_path;
  ROS_INFO("%s", str.c_str());
  cv::Mat target_img = cv::imread(str.c_str(), CV_LOAD_IMAGE_COLOR);  // Read the file

  if (!target_img.data)  // Check for invalid input
  {
    ROS_INFO("Could not open or find the image");
    return false;
  };

  image = target_img;

  centerAndCorners.resize(5);
  centerAndCorners[0] = cv::Point(0, 0);
  centerAndCorners[1] = cv::Point(image.cols, 0);
  centerAndCorners[2] = cv::Point(image.cols, image.rows);
  centerAndCorners[3] = cv::Point(0, image.rows);
  centerAndCorners[4] = cv::Point(image.cols / 2.0, image.rows / 2.0);

  ROS_DEBUG("TARGET IMAGE DIMENSIONS: %d,%d", image.cols, image.rows);

  detector.detect(image, keypoints);
  extractor.compute(image, keypoints, descriptors);
  /*
  std::vector<cv::Mat> descriptor_vector;
  descriptor_vector.push_back(descriptors);
  matcher.add(descriptor_vector);
  matcher.train();
  */
  ROS_DEBUG("end Target::init");
  ROS_DEBUG("Target: %d keypoints", keypoints.size());
  return true;
}

bool Target::detect(cv::Mat cam_descriptors, std::vector< cv::DMatch >& good_matches
#ifdef DEBUG_PROJECTION
                    ,
                    std::vector< cv::KeyPoint >& cam_keypoints, ucl_drone::Pose3D& pose
#endif
                    )
{
  ROS_DEBUG("begin Target::detect");
  // step 3: matching descriptors
  std::vector< cv::DMatch > matches;
  if (cam_descriptors.rows < descriptors.rows / 4.0)
  {
    return false;
  }
  matcher.match(descriptors, cam_descriptors, matches);

  // step 5: Draw only "good" matches
  // (i.e. whose distance is less than 3*min_dist )
  int j = 0;
  for (int i = 0; i < descriptors.rows; i++)
  {
    if (matches[i].distance < 200.0)
    // TODO: distance threshold: 250.0 for a given target ... --> parameter?
    {
      good_matches.push_back(matches[i]);
      j++;
    }
  }
  if (j < descriptors.rows / 6.0)
    return false;

#ifdef DEBUG_PROJECTION

  std::vector< cv::Point2f > obj;
  std::vector< cv::Point2f > scene;

  for (int i = 0; i < good_matches.size(); i++)
  {
    // Get the keypoints from the good matches
    obj.push_back(keypoints[good_matches[i].queryIdx].pt);
    scene.push_back(cam_keypoints[good_matches[i].trainIdx].pt);
  }

  std::vector< uchar > mask;
  cv::Mat H =
      cv::findHomography(obj, scene, CV_RANSAC, 2, mask);  // arg#4: ransacReprojThreshold [pixel]

  std::string rel_error = "\nrelative_error = [";
  std::string abs_error = "\nabsolute_error = [";
  for (int i = 0; i < good_matches.size(); i++)
  {
    for (int j = i + 1; j < good_matches.size(); j++)
    {
      if (mask[i] == 1 && mask[j] == 1)
      {
        cv::Point2f target_point1 = this->keypoints[good_matches[i].queryIdx].pt;
        cv::Point2f target_point2 = this->keypoints[good_matches[j].queryIdx].pt;
        double dist1 = cv::norm(target_point1 - target_point2) * 0.274 / 855.0;  // 0.205 / 135.0;

        if (dist1 > 0.1)
        {
          std::vector< cv::Point2f > camera_points(2);
          camera_points[0] = cam_keypoints[good_matches[i].trainIdx].pt;
          camera_points[1] = cam_keypoints[good_matches[j].trainIdx].pt;
          std::vector< cv::Point3f > map_points(2);
          projection_2D(camera_points, pose, map_points, true);

          double dist2 = cv::norm(map_points[0] - map_points[1]);

          //   if (dist2 > 0.001)  // if two super-imposed identical descriptors in map
          //   {
          double relative_error = (dist2 - dist1) / dist1;

          double absolute_error = (dist2 - dist1);

          rel_error += to_string(relative_error) + ", ";
          abs_error += to_string(absolute_error) + ", ";
          //   }
        }
      }
    }
  }
  rel_error += "];\n\n";
  abs_error += "];\n\n";

  printf("%s", rel_error.c_str());
  printf("%s", abs_error.c_str());

#endif

  ROS_DEBUG("end Target::detect");
  return true;
}

void Target::draw(cv::Mat cam_img, std::vector< cv::KeyPoint > cam_keypoints,
                  std::vector< cv::DMatch > good_matches, cv::Mat& img_matches)
{
  cv::drawMatches(image, keypoints, cam_img, cam_keypoints, good_matches, img_matches,
                  cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector< char >(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  // step 6: Localize the object
  std::vector< cv::Point2f > obj;
  std::vector< cv::Point2f > scene;

  for (int i = 0; i < good_matches.size(); i++)
  {
    // Get the keypoints from the good matches
    obj.push_back(keypoints[good_matches[i].queryIdx].pt);
    scene.push_back(cam_keypoints[good_matches[i].trainIdx].pt);
  }

  cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC, 2);

  // step 7: Get the corners from the target
  std::vector< cv::Point2f > scene_corners(4);

  cv::perspectiveTransform(centerAndCorners, scene_corners, H);

  // step 8: Draw lines between the corners (the mapped object in the scene - image_2 )
  cv::line(img_matches, scene_corners[0] + centerAndCorners[1],
           scene_corners[1] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(img_matches, scene_corners[1] + centerAndCorners[1],
           scene_corners[2] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(img_matches, scene_corners[2] + centerAndCorners[1],
           scene_corners[3] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(img_matches, scene_corners[3] + centerAndCorners[1],
           scene_corners[0] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);

  ROS_DEBUG("corner[0] %f %f", scene_corners[0].x, scene_corners[0].y);
  ROS_DEBUG("corner[1] %f %f", scene_corners[1].x, scene_corners[1].y);
  ROS_DEBUG("corner[2] %f %f", scene_corners[2].x, scene_corners[2].y);
  ROS_DEBUG("corner[3] %f %f", scene_corners[3].x, scene_corners[3].y);

  ROS_DEBUG("end Target::draw");
}

void Target::position(std::vector< cv::KeyPoint > cam_keypoints,
                      std::vector< cv::DMatch > good_matches, std::vector< cv::Point2f >& coord)
{
  // step 6: Localize the object
  std::vector< cv::Point2f > obj;
  std::vector< cv::Point2f > scene;

  for (int i = 0; i < good_matches.size(); i++)
  {
    // Get the keypoints from the good matches
    obj.push_back(keypoints[good_matches[i].queryIdx].pt);
    scene.push_back(cam_keypoints[good_matches[i].trainIdx].pt);
  }

  cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC, 2);

  // step 7: Get the corners from the target
  coord.resize(5);

  cv::perspectiveTransform(centerAndCorners, coord, H);

  ROS_DEBUG("end Target::position");
}
