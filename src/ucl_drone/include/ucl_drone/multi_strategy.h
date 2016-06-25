/*!
 *  \file multi_strategy.h
 *  \brief Basic strategy for multi-agent flight. Specify the mission and the
 * drone roles.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Multi-Agent strategy ROS node for the specific mission.
 * Contains:
 *        -
 *        -
 *        -
 *        -
 */

#ifndef UCL_DRONE_MULTI_STRATEGY_H
#define UCL_DRONE_MULTI_STRATEGY_H

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

// messages
#include <ucl_drone/DroneRole.h>
#include <ucl_drone/DroneRoles.h>
#include <ucl_drone/Pose3D.h>

// ucl_drone
#include <ucl_drone/drone_role.h>
#include <ucl_drone/ucl_drone.h>

/*!
 *  \class MultiStrategy
 *  \brief Provide tools to let drones communicate a common strategy
 */
class MultiStrategy
{
private:
  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber pose_dr1_sub;
  std::string pose_dr1_channel;

  ros::Subscriber pose_dr2_sub;
  std::string pose_dr2_channel;

  // Publishers
  ros::Publisher drones_roles_pub;
  std::string drones_roles_channel;

  //! \brief Callback when pose of drone 1 is received
  void pose_dr1_Cb(const ucl_drone::Pose3D::ConstPtr posePtr);

  //! \brief Callback when pose of drone 1 is received
  void pose_dr2_Cb(const ucl_drone::Pose3D::ConstPtr posePtr);

  //! Measure
  ucl_drone::Pose3D lastPoseReceived_dr1;
  ucl_drone::Pose3D lastPoseReceived_dr2;

  //! Frequently updated list of drone roles
  std::vector< DroneRole > role_list;

public:
  //! \brief Contructor.
  MultiStrategy();

  //! \brief Destructor.
  ~MultiStrategy();

  void init();

  void PublishDroneRole();

  bool pose_publishing_dr1;

  bool pose_publishing_dr2;
};

#endif /* UCL_DRONE_MULTI_STRATEGY_H */
