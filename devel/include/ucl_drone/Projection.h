// Generated by gencpp from file ucl_drone/Projection.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_PROJECTION_H
#define UCL_DRONE_MESSAGE_PROJECTION_H

#include <ros/service_traits.h>


#include <ucl_drone/ProjectionRequest.h>
#include <ucl_drone/ProjectionResponse.h>


namespace ucl_drone
{

struct Projection
{

typedef ProjectionRequest Request;
typedef ProjectionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Projection
} // namespace ucl_drone


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ucl_drone::Projection > {
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::ucl_drone::Projection&) { return value(); }
};

template<>
struct DataType< ::ucl_drone::Projection > {
  static const char* value()
  {
    return "ucl_drone/Projection";
  }

  static const char* value(const ::ucl_drone::Projection&) { return value(); }
};


// service_traits::MD5Sum< ::ucl_drone::ProjectionRequest> should match 
// service_traits::MD5Sum< ::ucl_drone::Projection > 
template<>
struct MD5Sum< ::ucl_drone::ProjectionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ucl_drone::Projection >::value();
  }
  static const char* value(const ::ucl_drone::ProjectionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ucl_drone::ProjectionRequest> should match 
// service_traits::DataType< ::ucl_drone::Projection > 
template<>
struct DataType< ::ucl_drone::ProjectionRequest>
{
  static const char* value()
  {
    return DataType< ::ucl_drone::Projection >::value();
  }
  static const char* value(const ::ucl_drone::ProjectionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ucl_drone::ProjectionResponse> should match 
// service_traits::MD5Sum< ::ucl_drone::Projection > 
template<>
struct MD5Sum< ::ucl_drone::ProjectionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ucl_drone::Projection >::value();
  }
  static const char* value(const ::ucl_drone::ProjectionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ucl_drone::ProjectionResponse> should match 
// service_traits::DataType< ::ucl_drone::Projection > 
template<>
struct DataType< ::ucl_drone::ProjectionResponse>
{
  static const char* value()
  {
    return DataType< ::ucl_drone::Projection >::value();
  }
  static const char* value(const ::ucl_drone::ProjectionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // UCL_DRONE_MESSAGE_PROJECTION_H
