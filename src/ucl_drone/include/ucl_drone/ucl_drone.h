/*!
 *  \file ucl_drone.h
 *  \brief Common definition for all nodes in the package ucl_drone
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 */

#ifndef UCL_DRONE_H
#define UCL_DRONE_H

#define PI 3.14159265

//! Role code definitions (multistrategy)
#define EMERGENCY_STOP 0
#define STAY_IDDLE 1
#define GO_TO 2
#define EXPLORE_AND_MAP 3
#define EXPLORE_UNTIL_TARGET 4

//! Role code definitions (strategy)
#define WAIT 0
#define TAKE_OFF 1
#define SEEK 2
#define GOTO 3
#define LAND 4
#define FOLLOW 5
#define BACK_HOME 6

// ! Room dimensions (in meters) (pathplanning)
#define SIDE 3

// camera parameters
#define IMG_WIDTH 735.0  // 640.0
#define IMG_HEIGHT 360.0
#define IMG_CENTER_x 350.6  // 305.0
#define IMG_CENTER_y 182.2
#define FOCAL_LENGTH_x 529.1  // 461.3  // 564.58 // à encore tuner
#define FOCAL_LENGTH_y 529.1  // 563.20 // à encore tuner
// camera matrix after re-calibration (re-calib after correction) see file
// 're_calib_after_correcion' besides the ardrone_front.yaml. Carefull! in
// ardrone_front.yaml, // stands under the name 'projection', becasue
// 'camera matrix' stands for the matrix BEFORE undistorsion by the image_proc node

template < class T >
std::string to_string(T i)
{
  std::stringstream ss;
  std::string s;
  ss << i;
  s = ss.str();

  return s;
}

#endif /* UCL_DRONE_H */
