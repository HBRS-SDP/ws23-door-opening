// Generated by gencpp from file kortex_driver/GetControllerConfigurationModeResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETCONTROLLERCONFIGURATIONMODERESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETCONTROLLERCONFIGURATIONMODERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ControllerConfigurationMode.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetControllerConfigurationModeResponse_
{
  typedef GetControllerConfigurationModeResponse_<ContainerAllocator> Type;

  GetControllerConfigurationModeResponse_()
    : output()  {
    }
  GetControllerConfigurationModeResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ControllerConfigurationMode_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetControllerConfigurationModeResponse_

typedef ::kortex_driver::GetControllerConfigurationModeResponse_<std::allocator<void> > GetControllerConfigurationModeResponse;

typedef boost::shared_ptr< ::kortex_driver::GetControllerConfigurationModeResponse > GetControllerConfigurationModeResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetControllerConfigurationModeResponse const> GetControllerConfigurationModeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c13bb07c2a93909cdcd58a87998a10d6";
  }

  static const char* value(const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc13bb07c2a93909cULL;
  static const uint64_t static_value2 = 0xdcd58a87998a10d6ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetControllerConfigurationModeResponse";
  }

  static const char* value(const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ControllerConfigurationMode output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerConfigurationMode\n"
"\n"
"bool enable\n"
;
  }

  static const char* value(const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetControllerConfigurationModeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetControllerConfigurationModeResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::ControllerConfigurationMode_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETCONTROLLERCONFIGURATIONMODERESPONSE_H
