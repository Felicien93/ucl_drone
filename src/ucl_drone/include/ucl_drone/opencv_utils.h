/*!
 *  \file opencv_utils.h
 *  \brief opencv_utils
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone.
 */

#ifndef UCL_DRONE_OPENCV_UTILS_H
#define UCL_DRONE_OPENCV_UTILS_H

#include <ros/ros.h>

//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/nonfree/nonfree.hpp>

#include <ucl_drone/PointXYZRGBSIFT.h>

cv::Mat rotationMatrixX(const double angle);
cv::Mat rotationMatrixY(const double angle);
cv::Mat rotationMatrixZ(const double angle);
cv::Mat rollPitchYawToRotationMatrix(const double roll, const double pitch, const double yaw);
cv::Mat rTMatrix(const cv::Mat rot, const double tx, const double ty, const double tz);
void debugRTMatrix(cv::Mat Rt);

template < typename T >
std::vector< cv::Point_< T > > Points(const std::vector< cv::KeyPoint >& keypoints);

std::vector< pcl::PointXYZRGBSIFT > Points(const std::vector< cv::KeyPoint >& keypoints);

#endif /* UCL_DRONE_OPENCV_UTILS_H */
