// Generated by gencpp from file kortex_driver/DeleteAllSequenceTasks.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_DELETEALLSEQUENCETASKS_H
#define KORTEX_DRIVER_MESSAGE_DELETEALLSEQUENCETASKS_H

#include <ros/service_traits.h>


#include <kortex_driver/DeleteAllSequenceTasksRequest.h>
#include <kortex_driver/DeleteAllSequenceTasksResponse.h>


namespace kortex_driver
{

struct DeleteAllSequenceTasks
{

typedef DeleteAllSequenceTasksRequest Request;
typedef DeleteAllSequenceTasksResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DeleteAllSequenceTasks
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::DeleteAllSequenceTasks > {
  static const char* value()
  {
    return "04dfaeca45772f660e0913aa84774e13";
  }

  static const char* value(const ::kortex_driver::DeleteAllSequenceTasks&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::DeleteAllSequenceTasks > {
  static const char* value()
  {
    return "kortex_driver/DeleteAllSequenceTasks";
  }

  static const char* value(const ::kortex_driver::DeleteAllSequenceTasks&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::DeleteAllSequenceTasksRequest> should match
// service_traits::MD5Sum< ::kortex_driver::DeleteAllSequenceTasks >
template<>
struct MD5Sum< ::kortex_driver::DeleteAllSequenceTasksRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::DeleteAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::DeleteAllSequenceTasksRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::DeleteAllSequenceTasksRequest> should match
// service_traits::DataType< ::kortex_driver::DeleteAllSequenceTasks >
template<>
struct DataType< ::kortex_driver::DeleteAllSequenceTasksRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::DeleteAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::DeleteAllSequenceTasksRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::DeleteAllSequenceTasksResponse> should match
// service_traits::MD5Sum< ::kortex_driver::DeleteAllSequenceTasks >
template<>
struct MD5Sum< ::kortex_driver::DeleteAllSequenceTasksResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::DeleteAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::DeleteAllSequenceTasksResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::DeleteAllSequenceTasksResponse> should match
// service_traits::DataType< ::kortex_driver::DeleteAllSequenceTasks >
template<>
struct DataType< ::kortex_driver::DeleteAllSequenceTasksResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::DeleteAllSequenceTasks >::value();
  }
  static const char* value(const ::kortex_driver::DeleteAllSequenceTasksResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_DELETEALLSEQUENCETASKS_H
