

#ifndef UCL_DRONE_IMAGE_PROCESSOR_H
#define UCL_DRONE_IMAGE_PROCESSOR_H

#include <ucl_drone/computer_vision/computer_vision.h>
/*  \class ImageProcessor
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
  ros::Publisher processed_image_pub;
  std::string processed_image_channel_out;

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
  sensor_msgs::Image::ConstPtr lastImageReceived;

public:
  //! \brief Contructor.
  ImageProcessor();

  //! \brief Destructor.
  ~ImageProcessor();

  void publishProcessedImg();
  bool pose_publishing;
  bool video_publishing;
};

#endif /*UCL_DRONE_IMAGE_PROCESSOR_H*/
