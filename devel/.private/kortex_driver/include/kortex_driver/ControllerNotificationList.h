// Generated by gencpp from file kortex_driver/ControllerNotificationList.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_CONTROLLERNOTIFICATIONLIST_H
#define KORTEX_DRIVER_MESSAGE_CONTROLLERNOTIFICATIONLIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ControllerNotification.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ControllerNotificationList_
{
  typedef ControllerNotificationList_<ContainerAllocator> Type;

  ControllerNotificationList_()
    : notifications()  {
    }
  ControllerNotificationList_(const ContainerAllocator& _alloc)
    : notifications(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::kortex_driver::ControllerNotification_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::kortex_driver::ControllerNotification_<ContainerAllocator> >> _notifications_type;
  _notifications_type notifications;





  typedef boost::shared_ptr< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> const> ConstPtr;

}; // struct ControllerNotificationList_

typedef ::kortex_driver::ControllerNotificationList_<std::allocator<void> > ControllerNotificationList;

typedef boost::shared_ptr< ::kortex_driver::ControllerNotificationList > ControllerNotificationListPtr;
typedef boost::shared_ptr< ::kortex_driver::ControllerNotificationList const> ControllerNotificationListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ControllerNotificationList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ControllerNotificationList_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerNotificationList_<ContainerAllocator2> & rhs)
{
  return lhs.notifications == rhs.notifications;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ControllerNotificationList_<ContainerAllocator1> & lhs, const ::kortex_driver::ControllerNotificationList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3d9f337a9914809bd100d1e4faaea316";
  }

  static const char* value(const ::kortex_driver::ControllerNotificationList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3d9f337a9914809bULL;
  static const uint64_t static_value2 = 0xd100d1e4faaea316ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ControllerNotificationList";
  }

  static const char* value(const ::kortex_driver::ControllerNotificationList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"ControllerNotification[] notifications\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerNotification\n"
"\n"
"Timestamp timestamp\n"
"UserProfileHandle user_handle\n"
"Connection connection\n"
"ControllerNotification_state oneof_state\n"
"================================================================================\n"
"MSG: kortex_driver/Timestamp\n"
"\n"
"uint32 sec\n"
"uint32 usec\n"
"================================================================================\n"
"MSG: kortex_driver/UserProfileHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/Connection\n"
"\n"
"UserProfileHandle user_handle\n"
"string connection_information\n"
"uint32 connection_identifier\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerNotification_state\n"
"\n"
"ControllerState[] controller_state\n"
"ControllerElementState[] controller_element\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerState\n"
"\n"
"ControllerHandle handle\n"
"uint32 event_type\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerHandle\n"
"\n"
"uint32 type\n"
"uint32 controller_identifier\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerElementState\n"
"\n"
"ControllerElementHandle handle\n"
"uint32 event_type\n"
"float32 axis_value\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerElementHandle\n"
"\n"
"ControllerHandle controller_handle\n"
"ControllerElementHandle_identifier oneof_identifier\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerElementHandle_identifier\n"
"\n"
"uint32[] button\n"
"uint32[] axis\n"
;
  }

  static const char* value(const ::kortex_driver::ControllerNotificationList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.notifications);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControllerNotificationList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ControllerNotificationList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ControllerNotificationList_<ContainerAllocator>& v)
  {
    s << indent << "notifications[]" << std::endl;
    for (size_t i = 0; i < v.notifications.size(); ++i)
    {
      s << indent << "  notifications[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kortex_driver::ControllerNotification_<ContainerAllocator> >::stream(s, indent + "    ", v.notifications[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_CONTROLLERNOTIFICATIONLIST_H
