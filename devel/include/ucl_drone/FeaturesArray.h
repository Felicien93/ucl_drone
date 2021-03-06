// Generated by gencpp from file ucl_drone/FeaturesArray.msg
// DO NOT EDIT!


#ifndef UCL_DRONE_MESSAGE_FEATURESARRAY_H
#define UCL_DRONE_MESSAGE_FEATURESARRAY_H

#include <ros/service_traits.h>


#include <ucl_drone/FeaturesArrayRequest.h>
#include <ucl_drone/FeaturesArrayResponse.h>


namespace ucl_drone
{

struct FeaturesArray
{

typedef FeaturesArrayRequest Request;
typedef FeaturesArrayResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FeaturesArray
} // namespace ucl_drone


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ucl_drone::FeaturesArray > {
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::ucl_drone::FeaturesArray&) { return value(); }
};

template<>
struct DataType< ::ucl_drone::FeaturesArray > {
  static const char* value()
  {
    return "ucl_drone/FeaturesArray";
  }

  static const char* value(const ::ucl_drone::FeaturesArray&) { return value(); }
};


// service_traits::MD5Sum< ::ucl_drone::FeaturesArrayRequest> should match 
// service_traits::MD5Sum< ::ucl_drone::FeaturesArray > 
template<>
struct MD5Sum< ::ucl_drone::FeaturesArrayRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ucl_drone::FeaturesArray >::value();
  }
  static const char* value(const ::ucl_drone::FeaturesArrayRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ucl_drone::FeaturesArrayRequest> should match 
// service_traits::DataType< ::ucl_drone::FeaturesArray > 
template<>
struct DataType< ::ucl_drone::FeaturesArrayRequest>
{
  static const char* value()
  {
    return DataType< ::ucl_drone::FeaturesArray >::value();
  }
  static const char* value(const ::ucl_drone::FeaturesArrayRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ucl_drone::FeaturesArrayResponse> should match 
// service_traits::MD5Sum< ::ucl_drone::FeaturesArray > 
template<>
struct MD5Sum< ::ucl_drone::FeaturesArrayResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ucl_drone::FeaturesArray >::value();
  }
  static const char* value(const ::ucl_drone::FeaturesArrayResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ucl_drone::FeaturesArrayResponse> should match 
// service_traits::DataType< ::ucl_drone::FeaturesArray > 
template<>
struct DataType< ::ucl_drone::FeaturesArrayResponse>
{
  static const char* value()
  {
    return DataType< ::ucl_drone::FeaturesArray >::value();
  }
  static const char* value(const ::ucl_drone::FeaturesArrayResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // UCL_DRONE_MESSAGE_FEATURESARRAY_H
