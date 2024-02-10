// Generated by gencpp from file kortex_driver/SetSafetyConfiguration.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETSAFETYCONFIGURATION_H
#define KORTEX_DRIVER_MESSAGE_SETSAFETYCONFIGURATION_H

#include <ros/service_traits.h>


#include <kortex_driver/SetSafetyConfigurationRequest.h>
#include <kortex_driver/SetSafetyConfigurationResponse.h>


namespace kortex_driver
{

struct SetSafetyConfiguration
{

typedef SetSafetyConfigurationRequest Request;
typedef SetSafetyConfigurationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetSafetyConfiguration
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetSafetyConfiguration > {
  static const char* value()
  {
    return "b1a9f41cb42ab39cba6bfb32d20fea4f";
  }

  static const char* value(const ::kortex_driver::SetSafetyConfiguration&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetSafetyConfiguration > {
  static const char* value()
  {
    return "kortex_driver/SetSafetyConfiguration";
  }

  static const char* value(const ::kortex_driver::SetSafetyConfiguration&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetSafetyConfigurationRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetSafetyConfiguration >
template<>
struct MD5Sum< ::kortex_driver::SetSafetyConfigurationRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetSafetyConfigurationRequest> should match
// service_traits::DataType< ::kortex_driver::SetSafetyConfiguration >
template<>
struct DataType< ::kortex_driver::SetSafetyConfigurationRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyConfigurationRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetSafetyConfigurationResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetSafetyConfiguration >
template<>
struct MD5Sum< ::kortex_driver::SetSafetyConfigurationResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyConfigurationResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetSafetyConfigurationResponse> should match
// service_traits::DataType< ::kortex_driver::SetSafetyConfiguration >
template<>
struct DataType< ::kortex_driver::SetSafetyConfigurationResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetSafetyConfiguration >::value();
  }
  static const char* value(const ::kortex_driver::SetSafetyConfigurationResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETSAFETYCONFIGURATION_H
