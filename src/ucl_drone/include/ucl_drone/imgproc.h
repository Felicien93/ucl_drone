/*!
 *  \file imgproc.h
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

#ifndef UCL_DRONE_IMGPROC_H
#define UCL_DRONE_IMGPROC_H

#include <boost/atomic.hpp>
//#include <mutex>
#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

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

// cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>  // pcl::PointCloud

// messages
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/image_encodings.h>
#include <ucl_drone/Pose3D.h>

// ucl_drone
#include <ucl_drone/map.h>
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/ucl_drone.h>

//! Name of the window where images are displayed
static const std::string OPENCV_WINDOW1 = "Keypoints window";

//! Name of the window where images are displayed
static const std::string OPENCV_WINDOW2 = "Object detection";

//! Filename to the target from within the package
static const std::string TARGET_RELPATH = "/target/target.jpg";

static const cv::SiftFeatureDetector detector;
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
  void draw(cv::Mat cam_img, std::vector< cv::KeyPoint > cam_keypoints, std::vector< cv::DMatch > good_matches,
            cv::Mat& img_matches);
};

/*!
 *  \class ProcessedImage
 *  \brief Provide tools to manipulate an camera image
 */
class ProcessedImage
{
public:
  cv_bridge::CvImagePtr cv_ptr;
  std::vector< cv::KeyPoint > keypoints;
  cv::Mat descriptors;
  ucl_drone::Pose3D pose;
  ros::Time stamp;

  ProcessedImage();
  ~ProcessedImage();

  void init(const sensor_msgs::Image image, const ucl_drone::Pose3D pose);

  void drawKeypoints(cv::Mat& output);
};

/*!
 *  \class Synchro
 *  \brief multithread synchronizer (provide request system)
 */
class Synchro
{
private:
  static bool callback_flag;
  static std::vector< Synchro* > instances_list;
  bool thread_flag;
  boost::shared_ptr< ProcessedImage > response;

  static boost::mutex callback_mutex;
  boost::mutex thread_mutex;

public:
  Synchro();
  ~Synchro();
  static bool getCallbackFlag();
  bool getThreadFlag();
  void sendRequest();
  boost::shared_ptr< ProcessedImage > getResponse();
  static void sendResponse(const boost::shared_ptr< ProcessedImage > cam_img);
};

/** \class ImageProcessor
 *  \brief Class of the image processor node for ROS.
 */
class ImageProcessor
{
private:
  ros::NodeHandle nh_;

  // Subscribers
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::string video_channel_;
  ros::Subscriber pose_sub;
  std::string pose_channel;

  // Publishers
  ros::Publisher pointcloud_pub;
  std::string pointcloud_channel_out;

  //! true if the target is successfully loaded
  bool target_loaded;

  Target target;

  //! \brief Callback when image is received
  //!
  //! This function is called each time a new image is published
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);

  void poseCb(const ucl_drone::Pose3D::ConstPtr posePtr);

  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr);

  //! Measure
  ucl_drone::Pose3D lastPoseReceived;
  ardrone_autonomy::Navdata lastNavdataReceived;

public:
  //! \brief Contructor.
  ImageProcessor();

  //! \brief Destructor.
  ~ImageProcessor();

  //! \brief Loads the image of the target and preprocess
  void loadTarget();

  //! Send points to Map
  //! \todo not implemented
  bool sendToMap(pcl::PointCloud< pcl::PointXYZRGB >::Ptr new_points);

  // thread functions
  void guiDrawKeypoints(Synchro* synchro);
  void guiDrawTarget(Synchro* synchro);
  void triangulationThreadFunc(Synchro* synchro);

  //! Call opencv triangulate
  //! /param[out] points_out: points in camera 1 frame
  void triangulate(boost::shared_ptr< ProcessedImage > image1, boost::shared_ptr< ProcessedImage > image2,
                   pcl::PointCloud< pcl::PointXYZRGB >::Ptr points_out);
};

/*!
 *  \class MovementEstimatorFromDescriptors
 *  \brief Provide tools to estimate the movement between two sets of keypoints descriptors
 *  \todo not implemented
 */
class MovementEstimatorFromDescriptors
{
private:
public:
  //! Constructor. Initialize an empty estimator
  MovementEstimatorFromDescriptors();

  //! Destructor.
  ~MovementEstimatorFromDescriptors();
};

/*!
 *  \class PoseEstimatorFromMap
 *  \brief Provide the tools to estimate the pose of a view (its keypoints descriptors) according to the information in
 * the map
 * \todo not implemented
 */
class PoseEstimatorFromMap
{
private:
  //! Client to request the service of the map to perform projection
  //! \see Map::requestProjection
  // ros::ServiceClient ProjectionClient;

public:
  //! Constructor. Initialize an empty estimator
  PoseEstimatorFromMap();

  //! Destructor.
  ~PoseEstimatorFromMap();
};

#endif /* UCL_DRONE_IMGPROC_H */
