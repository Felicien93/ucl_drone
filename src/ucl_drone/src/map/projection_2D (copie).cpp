/* Function convert_imgpoint_to_mappoint
*	@pre : Takes as inputs the pixel positions on the image(pixelx,y),
*			and the navdata.
*	@post : Pointers to the output map positions (output_mapx,y)
*	Converts a point from the image workspace to the map workspace.
*	This requires geometric conversions based on rotation matrices as well
*as
*	calibrated values FOV_X,y, the image dimensions (WIDTH and HEIGHT) all
*defined
*	in the header (.h)
*/

#include <ucl_drone/imgproc2D/imgproc2D.h>

#define MAP_WIDTH 640
#define MAP_HEIGHT 360
// Half of the total view angle for x and y respectively
#define FOV_X 0.2575
#define FOV_Y 0.465
// Measured 93cm along y axis and 51.5cm along x axis for height = 100cm

void ImageProcessor::convert_imgpoints_to_mappoints(
    boost::shared_ptr< ProcessedImage > image, ucl_drone::Pose3D pose,
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointcloud)
{
  double altitude = 1.3;  // image->navdata.altd; // Altitude given by the verticale

  double theta =
      (double)image->navdata.rotY / 1000 / 180 * PI;  // Conversion from 10^-3 degrees to radians
  double phi = -(double)image->navdata.rotX / 1000 / 180 * PI;
  double theta_z = (double)image->navdata.rotZ / 1000 / 180 * PI;
  double theta_x = cos(theta_z) * theta - sin(theta_z) * phi;
  double theta_y = -sin(theta_z) * theta - cos(theta_z) * phi;

  // Pixel projection on image plane
  // Distance from the drone to the center of the image plane (=center on the
  // ground)
  double plane_dist = altitude / cos(theta_x) / cos(theta_y);

  for (int i = 0; i < image->keypoints.size(); i++)
  {
    double pixelx = image->keypoints[i].pt.x;
    double pixely = image->keypoints[i].pt.y;
    /* (x,y,z) is the relative position between the camera and the point on the
    image
    !! Watch out, x and y axis are switched between the image and the map:
            Looking forwards is along positive y values on the image as opposed
    to
    positive
            x values on the map */
    double x = plane_dist *
               sin(((double)pixelx - (double)MAP_WIDTH / 2) / ((double)MAP_WIDTH / 2) * FOV_Y);
    double y = plane_dist *
               sin(-((double)pixely - (double)MAP_HEIGHT / 2) / ((double)MAP_HEIGHT / 2) * FOV_X);
    double z = 1 / cos(theta_x) / cos(theta_y);
    /* The terms plane_dist get eliminated by division, however if the depth of
    the points weren't the
    same, the results would change. So plane_dist is left in case 3D
    implementation is to be implemented
    at some point. */

    /* (dx,dy,dz) are results of multiplying (x,y,z) by three rotation matrices,
    respectively around axis x,y and z */
    double dx = cos(theta_y) * (sin(theta_z) * y + cos(theta_z) * x) - sin(theta_y) * z;
    double dy =
        sin(theta_x) * (cos(theta_y) * z + sin(theta_y) * (sin(theta_z) * y + cos(theta_z) * x)) +
        cos(theta_x) * (cos(theta_z) * y - sin(theta_z) * x);
    double dz =
        cos(theta_x) * (cos(theta_y) * z + sin(theta_y) * (sin(theta_z) * y + cos(theta_z) * x)) -
        sin(theta_x) * (cos(theta_z) * y - sin(theta_z) * x);

    // printf("dx = %f, dy = %f, dz = %f\n", dx, dy, dz);

    int rgb_ = 255 << 16 | 0 << 8 | 0;  // red
    pcl::PointXYZRGB point;
    point.x = pose.x + dx / dz;
    point.y = pose.y + dy / dz;
    point.z = 0;
    point.rgb = *reinterpret_cast< float* >(&rgb_);
    pointcloud->push_back(point);
  }
}
