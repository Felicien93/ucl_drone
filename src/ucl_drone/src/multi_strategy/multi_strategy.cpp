/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/multi_strategy.h>

MultiStrategy::MultiStrategy()
{
  // Subscribers
  pose_dr1_channel = nh_.resolveName("/ucl_drone_5/pose_estimation");
  pose_dr1_sub = nh_.subscribe(pose_dr1_channel, 1, &MultiStrategy::pose_dr1_Cb, this);

  pose_dr2_channel = nh_.resolveName("/ucl_drone_4/pose_estimation");
  pose_dr2_sub = nh_.subscribe(pose_dr2_channel, 1, &MultiStrategy::pose_dr2_Cb, this);

  // drone 1 feedback : ready or error or ...
  // drone 2 feedback : ready or error or ...

  // Publishers
  drones_roles_channel = nh_.resolveName("drones_roles");  // broadcast a list of roles
  drones_roles_pub = nh_.advertise< ucl_drone::DroneRoles >(drones_roles_channel, 1);

  // Useful initializations
  pose_publishing_dr1 = 0;
  pose_publishing_dr2 = 0;
}

MultiStrategy::~MultiStrategy()
{
}

void MultiStrategy::init()
{
  DroneRole role1("ucl_drone_5");
  role1.SetDroneRole(EXPLORE_UNTIL_TARGET);
  role_list.push_back(role1);
  DroneRole role2("ucl_drone_4");
  role2.SetDroneRole(GO_TO, "/ucl_drone_5/target_detected/");
  role_list.push_back(role2);
}

/* This function is called every time a new pose is published for drone 1*/
void MultiStrategy::pose_dr1_Cb(const ucl_drone::Pose3D::ConstPtr posePtr)
{
  lastPoseReceived_dr1 = *posePtr;
  pose_publishing_dr1 = true;
  // ROS_DEBUG("MultiStrategy::pose_dr1_Cb, %f", lastPoseReceived_dr1.x);
}

/* This function is called every time a new pose is published for drone 2*/
void MultiStrategy::pose_dr2_Cb(const ucl_drone::Pose3D::ConstPtr posePtr)
{
  lastPoseReceived_dr2 = *posePtr;
  pose_publishing_dr2 = true;
  // ROS_DEBUG("MultiStrategy::pose_dr2_Cb, %f", lastPoseReceived_dr2.x);
}

void MultiStrategy::PublishDroneRole()
{
  ucl_drone::DroneRoles msg = DroneRole::DroneRolesToMsg(role_list);
  drones_roles_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_strategy");

  /*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }*/

  MultiStrategy my_custom_strategy;
  ROS_DEBUG("end of MultiStrategy initialization");

  ros::Rate r1(3);        // 3Hz
  ros::Rate r2(1 / 5.0);  // 1/5Hz

  // while ((!my_custom_strategy.pose_publishing_dr1 || !my_custom_strategy.pose_publishing_dr2) &&
  //        ros::ok())
  // {
  //   ros::spinOnce();
  //   r1.sleep();
  // }
  // ROS_DEBUG("finished loop 1");

  // drone 1: begin mission "find target"

  // while mission done drone 1 flase -> wait (dont forget spin)

  // drone 2: begin mission goto "pose target" --> pass name of the publisher

  my_custom_strategy.init();
  // ros::spin();
  ROS_DEBUG("entering loop 2");
  while (ros::ok())
  {
    my_custom_strategy.PublishDroneRole();
    ros::spinOnce();
    r2.sleep();
    ROS_DEBUG("multistrategy has published drone role");
  }

  return 0;
}
