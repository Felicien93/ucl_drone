/*!
 *  \file pose_estimation.h
 *  \brief Pose estimation node of the drone (x,y,z,theta,phi,psi) in an
 *         absolute coordinate frame. At the present: on the basis of the
 *         Odometry computed in ardrone_autonomy. In future developpment:
 *         Kalman filters, visual odometry, raw sensors, etc.

 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Pose estimation ROS node for the
 *  ardrone. Contains:
 *            - pose Publisher
 *            - sensor fusion \todo{Kalman Filter}
 */

#ifndef UCL_POSE_ESTIMATION_H
#define UCL_POSE_ESTIMATION_H

// Header files
#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <signal.h>
//#include <tf/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ucl_drone
#include <ucl_drone/ucl_drone.h>

// messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <ucl_drone/Pose3D.h>

class PoseEstimator
{
private:
  ros::NodeHandle nh;

  ros::Subscriber navdata_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber odometry_sub;
  ros::Publisher pose_pub;
  ros::Subscriber reset_sub;

  std::string navdata_channel;
  std::string odometry_channel;
  std::string pose_channel;
  std::string imu_channel;
  std::string reset_channel;

  //! navdata
  ardrone_autonomy::Navdata lastNavdataReceived;

  //! imu data
  sensor_msgs::Imu lastImuReceived;

  //! odometry
  nav_msgs::Odometry lastOdometryReceived;

  ros::Time odom_time;
  float odometry_x;
  float odometry_y;
  double rot_Z_offset;
  double lastRotX;
  double lastRotY;
  double lastRotZ;

  //! Callback when navdata is received
  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr);

  //! Callback when imu is received
  void imuCb(const sensor_msgs::Imu::ConstPtr imuPtr);

  //! Callback when odometry is received
  void odometryCb(const nav_msgs::Odometry::ConstPtr odometryPtr);

public:
  // Constructor
  PoseEstimator();

  // Destructor
  ~PoseEstimator();

  bool odometry_publishing;
  void publish_pose();
  void reset(const std_msgs::Empty msg);
};

#endif /* UCL_POSE_ESTIMATION_H */
