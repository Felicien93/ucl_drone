// Generated by gencpp from file ucl_drone/cellUpdate.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_CELLUPDATE_H
#define UCL_DRONE_MESSAGE_CELLUPDATE_H


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
struct cellUpdate_
{
  typedef cellUpdate_<ContainerAllocator> Type;

  cellUpdate_()
    : header()
    , i(0.0)
    , j(0.0)
    , type(0.0)  {
    }
  cellUpdate_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , i(0.0)
    , j(0.0)
    , type(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _i_type;
  _i_type i;

   typedef double _j_type;
  _j_type j;

   typedef double _type_type;
  _type_type type;




  typedef boost::shared_ptr< ::ucl_drone::cellUpdate_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ucl_drone::cellUpdate_<ContainerAllocator> const> ConstPtr;

}; // struct cellUpdate_

typedef ::ucl_drone::cellUpdate_<std::allocator<void> > cellUpdate;

typedef boost::shared_ptr< ::ucl_drone::cellUpdate > cellUpdatePtr;
typedef boost::shared_ptr< ::ucl_drone::cellUpdate const> cellUpdateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ucl_drone::cellUpdate_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ucl_drone::cellUpdate_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ucl_drone::cellUpdate_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ucl_drone::cellUpdate_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::cellUpdate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ucl_drone::cellUpdate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::cellUpdate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ucl_drone::cellUpdate_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ucl_drone::cellUpdate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f2e33de50941fc2f2ebb9499a39b2f1b";
  }

  static const char* value(const ::ucl_drone::cellUpdate_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf2e33de50941fc2fULL;
  static const uint64_t static_value2 = 0x2ebb9499a39b2f1bULL;
};

template<class ContainerAllocator>
struct DataType< ::ucl_drone::cellUpdate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ucl_drone/cellUpdate";
  }

  static const char* value(const ::ucl_drone::cellUpdate_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ucl_drone::cellUpdate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#This message type is just used for a test\n\
\n\
Header header\n\
\n\
float64 i\n\
float64 j\n\
float64 type\n\
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

  static const char* value(const ::ucl_drone::cellUpdate_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ucl_drone::cellUpdate_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.i);
      stream.next(m.j);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct cellUpdate_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ucl_drone::cellUpdate_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ucl_drone::cellUpdate_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "i: ";
    Printer<double>::stream(s, indent + "  ", v.i);
    s << indent << "j: ";
    Printer<double>::stream(s, indent + "  ", v.j);
    s << indent << "type: ";
    Printer<double>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UCL_DRONE_MESSAGE_CELLUPDATE_H