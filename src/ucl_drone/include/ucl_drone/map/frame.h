/*!
 *  \file frame.h
 *  \brief Frame
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef UCL_DRONE_FRAME_H
#define UCL_DRONE_FRAME_H
#define PCL_NO_PRECOMPILE

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* Messages */
#include <sensor_msgs/image_encodings.h>

/* ucl_drone */
#include <ucl_drone/PointXYZRGBSIFT.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>
#include <ucl_drone/map/simple_map.h>

class Map;  // defined in ucl_drone/map/simple_map.h
            // declared here because ucl_drone/map/frame.h (current file)
            // is also included in ucl_drone/map/simple_map.h

/** \class Frame
 *  A description here
 */
class Frame
{
private:
  cv::Mat descriptors;

public:
  //! Contructor.
  Frame();

  //! Contructor.
  Frame(ucl_drone::ProcessedImageMsg::ConstPtr msg);

  //! Destructor.
  ~Frame();

  void matchWithMap(Map& map, std::vector< std::pair< int, int > >& idx_matching_points,
                    std::vector< int >& idx_unknown_points,
                    std::vector< cv::Point3f >& map_matching_points,
                    std::vector< cv::Point2f >& frame_matching_points,
                    std::vector< cv::Point2f >& frame_unknown_points);
  void convertToPcl(std::vector< int >& idx_points, std::vector< cv::Point3f > points_out,
                    pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud);
  void convertToPcl(std::vector< cv::Point3f > points_out,
                    pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud);

  ucl_drone::ProcessedImageMsg msg;
  std::vector< cv::Point2f > imgPoints;
};

#endif /* UCL_DRONE_FRAME_H */
