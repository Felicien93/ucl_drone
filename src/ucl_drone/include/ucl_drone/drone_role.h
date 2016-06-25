/*!
 *  \file drone_roles.h
 *  \brief Class definition to handle drone roles objects.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Multi-Agent strategy object destined to be transmited as
 *  a ROS message.Contains:
 *        -
 *        -
 *        -
 *        -
 */

#ifndef UCL_DRONE_DRONE_ROLES_H
#define UCL_DRONE_DRONE_ROLES_H

#include <stdarg.h>

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

// messages
#include <ucl_drone/DroneRole.h>
#include <ucl_drone/DroneRoles.h>
#include <ucl_drone/Pose3D.h>

// ucl_drone
#include <ucl_drone/ucl_drone.h>

/*!
 *  \class DroneRole
 *  \brief Class definition to handle drone roles objects.
 */
class DroneRole
{
private:
  //! Attributes
  int role;
  std::vector< std::string > parameters;
  std::string name;

public:
  //! \brief Contructor.
  //! \param[in] name String containing the name of the drone
  DroneRole(std::string name);

  //! \brief Destructor.
  ~DroneRole();

  void SetDroneRole(int role);
  void SetDroneRole(int role, std::vector< std::string > params);
  void SetDroneRole(int role, std::string param);
  void SetDroneRole(int role, int number_of_params, ...);

  int GetDroneRole();

  ucl_drone::DroneRole DroneRoleToMsg();

  void MsgToDroneRole();

  static ucl_drone::DroneRoles DroneRolesToMsg(std::vector< DroneRole > roles);
};

#endif /* UCL_DRONE_DRONE_ROLES_H */
