// Generated by gencpp from file kortex_driver/SetApiOptions.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETAPIOPTIONS_H
#define KORTEX_DRIVER_MESSAGE_SETAPIOPTIONS_H

#include <ros/service_traits.h>


#include <kortex_driver/SetApiOptionsRequest.h>
#include <kortex_driver/SetApiOptionsResponse.h>


namespace kortex_driver
{

struct SetApiOptions
{

typedef SetApiOptionsRequest Request;
typedef SetApiOptionsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetApiOptions
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SetApiOptions > {
  static const char* value()
  {
    return "11c85b567b00c6eb6cd55fdb8cb9ad1b";
  }

  static const char* value(const ::kortex_driver::SetApiOptions&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SetApiOptions > {
  static const char* value()
  {
    return "kortex_driver/SetApiOptions";
  }

  static const char* value(const ::kortex_driver::SetApiOptions&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SetApiOptionsRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SetApiOptions >
template<>
struct MD5Sum< ::kortex_driver::SetApiOptionsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetApiOptions >::value();
  }
  static const char* value(const ::kortex_driver::SetApiOptionsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetApiOptionsRequest> should match
// service_traits::DataType< ::kortex_driver::SetApiOptions >
template<>
struct DataType< ::kortex_driver::SetApiOptionsRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetApiOptions >::value();
  }
  static const char* value(const ::kortex_driver::SetApiOptionsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SetApiOptionsResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SetApiOptions >
template<>
struct MD5Sum< ::kortex_driver::SetApiOptionsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SetApiOptions >::value();
  }
  static const char* value(const ::kortex_driver::SetApiOptionsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SetApiOptionsResponse> should match
// service_traits::DataType< ::kortex_driver::SetApiOptions >
template<>
struct DataType< ::kortex_driver::SetApiOptionsResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SetApiOptions >::value();
  }
  static const char* value(const ::kortex_driver::SetApiOptionsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETAPIOPTIONS_H