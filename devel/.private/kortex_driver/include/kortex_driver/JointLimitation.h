// Generated by gencpp from file kortex_driver/JointLimitation.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_JOINTLIMITATION_H
#define KORTEX_DRIVER_MESSAGE_JOINTLIMITATION_H


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
struct JointLimitation_
{
  typedef JointLimitation_<ContainerAllocator> Type;

  JointLimitation_()
    : joint_identifier(0)
    , type(0)
    , value(0.0)  {
    }
  JointLimitation_(const ContainerAllocator& _alloc)
    : joint_identifier(0)
    , type(0)
    , value(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _joint_identifier_type;
  _joint_identifier_type joint_identifier;

   typedef uint32_t _type_type;
  _type_type type;

   typedef float _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::kortex_driver::JointLimitation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::JointLimitation_<ContainerAllocator> const> ConstPtr;

}; // struct JointLimitation_

typedef ::kortex_driver::JointLimitation_<std::allocator<void> > JointLimitation;

typedef boost::shared_ptr< ::kortex_driver::JointLimitation > JointLimitationPtr;
typedef boost::shared_ptr< ::kortex_driver::JointLimitation const> JointLimitationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::JointLimitation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::JointLimitation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::JointLimitation_<ContainerAllocator1> & lhs, const ::kortex_driver::JointLimitation_<ContainerAllocator2> & rhs)
{
  return lhs.joint_identifier == rhs.joint_identifier &&
    lhs.type == rhs.type &&
    lhs.value == rhs.value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::JointLimitation_<ContainerAllocator1> & lhs, const ::kortex_driver::JointLimitation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::JointLimitation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::JointLimitation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::JointLimitation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::JointLimitation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::JointLimitation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::JointLimitation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::JointLimitation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c7e8a86926cdf71a09bb6314d52e9586";
  }

  static const char* value(const ::kortex_driver::JointLimitation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc7e8a86926cdf71aULL;
  static const uint64_t static_value2 = 0x09bb6314d52e9586ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::JointLimitation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/JointLimitation";
  }

  static const char* value(const ::kortex_driver::JointLimitation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::JointLimitation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 joint_identifier\n"
"uint32 type\n"
"float32 value\n"
;
  }

  static const char* value(const ::kortex_driver::JointLimitation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::JointLimitation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_identifier);
      stream.next(m.type);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointLimitation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::JointLimitation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::JointLimitation_<ContainerAllocator>& v)
  {
    s << indent << "joint_identifier: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.joint_identifier);
    s << indent << "type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.type);
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_JOINTLIMITATION_H