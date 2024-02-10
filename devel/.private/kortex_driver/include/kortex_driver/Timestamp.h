// Generated by gencpp from file kortex_driver/Timestamp.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_TIMESTAMP_H
#define KORTEX_DRIVER_MESSAGE_TIMESTAMP_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_driver
{
template <class ContainerAllocator>
struct Timestamp_
{
  typedef Timestamp_<ContainerAllocator> Type;

  Timestamp_()
    : sec(0)
    , usec(0)  {
    }
  Timestamp_(const ContainerAllocator& _alloc)
    : sec(0)
    , usec(0)  {
  (void)_alloc;
    }



   typedef uint32_t _sec_type;
  _sec_type sec;

   typedef uint32_t _usec_type;
  _usec_type usec;





  typedef boost::shared_ptr< ::kortex_driver::Timestamp_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::Timestamp_<ContainerAllocator> const> ConstPtr;

}; // struct Timestamp_

typedef ::kortex_driver::Timestamp_<std::allocator<void> > Timestamp;

typedef boost::shared_ptr< ::kortex_driver::Timestamp > TimestampPtr;
typedef boost::shared_ptr< ::kortex_driver::Timestamp const> TimestampConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::Timestamp_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::Timestamp_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::Timestamp_<ContainerAllocator1> & lhs, const ::kortex_driver::Timestamp_<ContainerAllocator2> & rhs)
{
  return lhs.sec == rhs.sec &&
    lhs.usec == rhs.usec;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::Timestamp_<ContainerAllocator1> & lhs, const ::kortex_driver::Timestamp_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Timestamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::Timestamp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Timestamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::Timestamp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Timestamp_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::Timestamp_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::Timestamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "90da89d9efe7e712be65708e3d776fa4";
  }

  static const char* value(const ::kortex_driver::Timestamp_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x90da89d9efe7e712ULL;
  static const uint64_t static_value2 = 0xbe65708e3d776fa4ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::Timestamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/Timestamp";
  }

  static const char* value(const ::kortex_driver::Timestamp_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::Timestamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 sec\n"
"uint32 usec\n"
;
  }

  static const char* value(const ::kortex_driver::Timestamp_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::Timestamp_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sec);
      stream.next(m.usec);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Timestamp_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::Timestamp_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::Timestamp_<ContainerAllocator>& v)
  {
    s << indent << "sec: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sec);
    s << indent << "usec: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.usec);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_TIMESTAMP_H
