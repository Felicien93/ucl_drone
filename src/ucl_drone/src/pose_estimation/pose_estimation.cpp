/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include "ucl_drone/pose_estimation.h"

#include <math.h> /* isfinite*/

// Constructor
PoseEstimator::PoseEstimator()
{
  // Subsribers
  std::string drone_prefix;
  ros::param::get("~drone_prefix", drone_prefix);
  navdata_channel = nh.resolveName(drone_prefix + "ardrone/navdata");
  navdata_sub = nh.subscribe(navdata_channel, 10, &PoseEstimator::navdataCb, this);
  odometry_channel = nh.resolveName(drone_prefix + "ardrone/odometry");
  odometry_sub = nh.subscribe(odometry_channel, 10, &PoseEstimator::odometryCb, this);
  imu_channel = nh.resolveName(drone_prefix + "ardrone/imu");
  imu_sub = nh.subscribe(imu_channel, 10, &PoseEstimator::imuCb, this);

  reset_channel = nh.resolveName("reset_pose");
  reset_sub = nh.subscribe(reset_channel, 10, &PoseEstimator::reset, this);

  // Publishers
  pose_channel = nh.resolveName("pose_estimation");
  pose_pub = nh.advertise< ucl_drone::Pose3D >(pose_channel,
                                               1);  // 1 is size of output buffer (before send)

  // TODO : pose3D is in fact a geometry_msgs/Twist.msg (+velocities) --> check this

  odometry_publishing = false;
}

// Destructor
PoseEstimator::~PoseEstimator()
{
}

void PoseEstimator::reset(const std_msgs::Empty msg)
{
  // ros::ServiceClient client = nh.serviceClient< std_srvs::Empty
  // >("motherboard1/ardrone/flattrim");
  // std_srvs::Empty srv;
  // client.call(srv);

  odometry_x = 0;
  odometry_y = 0;
  odom_time = lastOdometryReceived.header.stamp;  //// a déjà reçu???

  double dummy1, dummy2;
  tf::Quaternion bt;
  tf::quaternionMsgToTF(lastOdometryReceived.pose.pose.orientation, bt);
  tf::Matrix3x3(bt).getRPY(dummy1, dummy2, rot_Z_offset);
  ROS_DEBUG("rot_Z_offset: %lf", rot_Z_offset);  //%d ?
}

void PoseEstimator::publish_pose()
{
  // instantiate the pose message
  ucl_drone::Pose3D pose_msg;

  // fill metadata

  // stupid recopying of odmometry
  // pose_msg.header.stamp = ros::Time().now();  // lastOdometryReceived.header.stamp;
  // pose_msg.header.frame_id = lastOdometryReceived.header.frame_id;
  // pose_msg.x = lastOdometryReceived.pose.pose.position.x;
  // pose_msg.y = lastOdometryReceived.pose.pose.position.y;
  // pose_msg.z = lastOdometryReceived.pose.pose.position.z;
  // tf::Quaternion bt;
  // tf::quaternionMsgToTF(lastOdometryReceived.pose.pose.orientation, bt);
  // tf::Matrix3x3(bt).getRPY(pose_msg.rotX, pose_msg.rotY, pose_msg.rotZ);
  // ROS_DEBUG("X: %f, Y: %f, Z: %f", pose_msg.rotX, pose_msg.rotY, pose_msg.rotZ);
  // pose_msg.xvel = lastOdometryReceived.twist.twist.linear.x;
  // pose_msg.yvel = lastOdometryReceived.twist.twist.linear.y;
  // pose_msg.zvel = lastOdometryReceived.twist.twist.linear.z;
  // pose_msg.rotXvel = lastOdometryReceived.twist.twist.angular.x;
  // pose_msg.rotYvel = lastOdometryReceived.twist.twist.angular.y;
  // pose_msg.rotZvel = lastOdometryReceived.twist.twist.angular.z;

  // our own odometry integrated from optical flow
  ros::Time previous_odom_time = odom_time;
  odom_time = lastOdometryReceived.header.stamp;
  double delta_t = (odom_time - previous_odom_time).toSec();
  if (delta_t == 0)  // it seems no new odometry was published
  {
    ROS_DEBUG("Is ardrone_autonomy always responding ?");
    return;
  }
  if (!std::isfinite(delta_t))
  {
    ROS_DEBUG("Something strange hapenned! delta_t is not finite!");
    return;
  }

  // Careful !!! In ardrone_odometry as well as in navdata, x and y non conventionnal (don't respect
  // the vectorial product). So we have to do x  = -x and y = -y
  pose_msg.rotX = lastRotX;
  pose_msg.rotY = lastRotY;
  pose_msg.rotZ = lastRotZ - (float)rot_Z_offset;  // rot_Z_offset determined by initial calibration
                                                   // (see PoseEstimator::reset)
  if (pose_msg.rotZ > M_PI)
    pose_msg.rotZ -= 2 * M_PI;
  if (pose_msg.rotZ < -M_PI)
    pose_msg.rotZ += 2 * M_PI;

  // Formulas recopied from ardrone_autonomy, to compute pose from integration
  // of the velocities
  odometry_x += (float)((cos(pose_msg.rotZ) * lastOdometryReceived.twist.twist.linear.x -
                         sin(pose_msg.rotZ) * lastOdometryReceived.twist.twist.linear.y) *
                        delta_t);
  odometry_y += (float)((sin(pose_msg.rotZ) * lastOdometryReceived.twist.twist.linear.x +
                         cos(pose_msg.rotZ) * lastOdometryReceived.twist.twist.linear.y) *
                        delta_t);

  pose_msg.header.stamp = odom_time;
  pose_msg.x = odometry_x;
  pose_msg.y = odometry_y;
  pose_msg.z = lastOdometryReceived.pose.pose.position.z;  // not received when drone not flying !!!
  pose_msg.xvel = lastOdometryReceived.twist.twist.linear.x;
  pose_msg.yvel = lastOdometryReceived.twist.twist.linear.y;
  pose_msg.zvel =
      lastOdometryReceived.twist.twist.linear.z;  // not received when drone not flying !!!
  pose_msg.rotXvel =
      lastOdometryReceived.twist.twist.angular.x;  // not received when drone not flying !!!
  pose_msg.rotYvel =
      lastOdometryReceived.twist.twist.angular.y;  // not received when drone not flying !!!
  pose_msg.rotZvel =
      lastOdometryReceived.twist.twist.angular.z;  // not received when drone not flying !!!

  // publish
  if (pose_pub.getNumSubscribers() > 0)
  {
    pose_pub.publish(pose_msg);
    ROS_DEBUG("ucl_drone::pose_estimation Pose published");
  }
}

void PoseEstimator::navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr)
{
  lastNavdataReceived = *navdataPtr;
}

void PoseEstimator::imuCb(const sensor_msgs::Imu::ConstPtr imuPtr)
{
  lastImuReceived = *imuPtr;
}

void PoseEstimator::odometryCb(const nav_msgs::Odometry::ConstPtr odometryPtr)
{
  ROS_DEBUG("odometry received");
  // to see what exactly is put in Odometry: ardrone_driver.cpp line 698 to 713

  double rotX, rotY, rotZ;
  tf::Quaternion bt;
  tf::quaternionMsgToTF(odometryPtr->pose.pose.orientation, bt);
  tf::Matrix3x3(bt).getRPY(rotX, rotY, rotZ);

  // avoid to publish NAN values
  if (std::isfinite(odometryPtr->twist.twist.linear.x) &&
      std::isfinite(odometryPtr->twist.twist.linear.y) && std::isfinite(rotX) &&
      std::isfinite(rotY) && std::isfinite(rotZ))
  // std::isfinite(odometryPtr->pose.pose.position.z) &&
  // std::isfinite(odometryPtr->pose.pose.orientation.x) &&
  // std::isfinite(odometryPtr->pose.pose.orientation.y) &&
  // std::isfinite(odometryPtr->pose.pose.orientation.z) &&
  // std::isfinite(odometryPtr->pose.pose.orientation.w) &&
  {
    odometry_publishing = true;
    lastOdometryReceived = *odometryPtr;
    lastRotX = rotX;
    lastRotY = rotY;
    lastRotZ = rotZ;
  }
  else
  {
    ROS_DEBUG("Is ardrone_autonomy odometry stupid Nan ?");
  }
}

int main(int argc, char** argv)
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
  ROS_INFO_STREAM("pose_estimation started!");
  ros::init(argc, argv, "pose_estimation");
  PoseEstimator myPose;
  ros::Rate r(20);

  ROS_DEBUG("pose estimation initialized");
  while (!myPose.odometry_publishing && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_DEBUG("ardrone_driver publishing: ending hibernate");
  // myPose.reset();  // do flat trim
  ROS_DEBUG("reset done");
  while (ros::ok())
  {
    myPose.publish_pose();
    ros::spinOnce();  // if we dont want this we have to place callback and services in threads
    r.sleep();
  }
  return 0;
}
