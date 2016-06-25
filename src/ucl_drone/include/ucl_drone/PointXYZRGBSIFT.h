/**
 *  \file mappoint.h
 *  \brief PointXYZWithDescriptor is a PointXYZ with a descriptor field of type cv::Mat to store the
 * (SIFT) descriptor
 * of the point (computed by opencv)
 */

/* ucl_basic_drone: basic_drone PointXYZWithDescriptor definition
 *
 * license ?
 * authors: Arnaud Jacques & Alexandre Leclère
 */

#ifndef UCL_DRONE_POINTXYZRGBSIFT_H
#define UCL_DRONE_POINTXYZRGBSIFT_H
#define PCL_NO_PRECOMPILE

//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/nonfree/nonfree.hpp>

namespace pcl
{
struct PointXYZRGBSIFT  //: PointXYZRGB
{
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_RGB;
  float descriptor[128];
  float view_count;

  //\todo{add a field to store the keyframe id}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZRGBSIFT,  // here we assume a XYZRGB + "descriptor"
                                                         // (as
                                                         // fields)
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(
                                      float[128], descriptor, descriptor)(float, view_count,
                                                                          view_count))

namespace pcl
{
struct PointUVSIFT  //: PointXYZRGB
{
  float u;
  float v;
  float descriptor[128];

  //\todo{add a field to store the keyframe id}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointUVSIFT,  // here we assume a XYZRGB + "descriptor"
                                                     // (as
                                                     // fields)
                                  (float, u, u)(float, v, v)(float[128], descriptor, descriptor))

#endif /* UCL_DRONE_POINTXYZRGBSIFT_H */
