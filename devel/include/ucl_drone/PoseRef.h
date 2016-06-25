// Generated by gencpp from file ucl_drone/PoseRef.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_POSEREF_H
#define UCL_DRONE_MESSAGE_POSEREF_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ucl_drone
{
template <class ContainerAllocator>
struct PoseRef_
{
  typedef PoseRef_<ContainerAllocator> Type;

  PoseRef_()
    : header()
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rotZ(0.0)
    , landAndStop(false)
    , takeoffAndStart(false)  {
    }
  PoseRef_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , rotZ(0.0)
    , landAndStop(false)
    , takeoffAndStart(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _rotZ_type;
  _rotZ_type rotZ;

   typedef uint8_t _landAndStop_type;
  _landAndStop_type landAndStop;

   typedef uint8_t _takeoffAndStart_type;
  _takeoffAndStart_type takeoffAndStart;




  typedef boost::shared_ptr< ::ucl_drone::PoseRef_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucl_drone::PoseRef_<ContainerAllocator> const> ConstPtr;

}; // struct PoseRef_

typedef ::ucl_drone::PoseRef_<std::allocator<void> > PoseRef;

typedef boost::shared_ptr< ::ucl_drone::PoseRef > PoseRefPtr;
typedef boost::shared_ptr< ::ucl_drone::PoseRef const> PoseRefConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucl_drone::PoseRef_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucl_drone::PoseRef_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ucl_drone

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'ucl_drone': ['/home/felicien/Desktop/Devel-Tracking/shared-ros-workspace/src/ucl_drone/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/home/felicien/Desktop/UCL_drones/shared-ros-workspace/src/ardrone_autonomy/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::PoseRef_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::PoseRef_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::PoseRef_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::PoseRef_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::PoseRef_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::PoseRef_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucl_drone::PoseRef_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d40f9037129845d22f4a15b255ed3109";
  }

  static const char* value(const ::ucl_drone::PoseRef_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd40f9037129845d2ULL;
  static const uint64_t static_value2 = 0x2f4a15b255ed3109ULL;
};

template<class ContainerAllocator>
struct DataType< ::ucl_drone::PoseRef_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucl_drone/PoseRef";
  }

  static const char* value(const ::ucl_drone::PoseRef_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucl_drone::PoseRef_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
# This represents an estimate of a position and velocity in 3D space.\n\
# The pose in this message should be specified in an absolute coordinate frame.\n\
\n\
# ucl definition of a desired pose message.\n\
\n\
Header header\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
float64 rotZ\n\
\n\
bool landAndStop\n\
bool takeoffAndStart\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::ucl_drone::PoseRef_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucl_drone::PoseRef_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.rotZ);
      stream.next(m.landAndStop);
      stream.next(m.takeoffAndStart);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct PoseRef_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucl_drone::PoseRef_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucl_drone::PoseRef_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "rotZ: ";
    Printer<double>::stream(s, indent + "  ", v.rotZ);
    s << indent << "landAndStop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.landAndStop);
    s << indent << "takeoffAndStart: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.takeoffAndStart);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCL_DRONE_MESSAGE_POSEREF_H
