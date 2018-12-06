// Generated by gencpp from file dart_delivery/RosieOdom.msg
// DO NOT EDIT!


#ifndef DART_DELIVERY_MESSAGE_ROSIEODOM_H
#define DART_DELIVERY_MESSAGE_ROSIEODOM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dart_delivery
{
template <class ContainerAllocator>
struct RosieOdom_
{
  typedef RosieOdom_<ContainerAllocator> Type;

  RosieOdom_()
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }
  RosieOdom_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;





  typedef boost::shared_ptr< ::dart_delivery::RosieOdom_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dart_delivery::RosieOdom_<ContainerAllocator> const> ConstPtr;

}; // struct RosieOdom_

typedef ::dart_delivery::RosieOdom_<std::allocator<void> > RosieOdom;

typedef boost::shared_ptr< ::dart_delivery::RosieOdom > RosieOdomPtr;
typedef boost::shared_ptr< ::dart_delivery::RosieOdom const> RosieOdomConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dart_delivery::RosieOdom_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dart_delivery::RosieOdom_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dart_delivery

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'dart_delivery': ['/home/robert/principles/project/catkin_ws/src/dart_delivery/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dart_delivery::RosieOdom_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dart_delivery::RosieOdom_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dart_delivery::RosieOdom_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dart_delivery::RosieOdom_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dart_delivery::RosieOdom_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dart_delivery::RosieOdom_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dart_delivery::RosieOdom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a130bc60ee6513855dc62ea83fcc5b20";
  }

  static const char* value(const ::dart_delivery::RosieOdom_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa130bc60ee651385ULL;
  static const uint64_t static_value2 = 0x5dc62ea83fcc5b20ULL;
};

template<class ContainerAllocator>
struct DataType< ::dart_delivery::RosieOdom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dart_delivery/RosieOdom";
  }

  static const char* value(const ::dart_delivery::RosieOdom_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dart_delivery::RosieOdom_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n\
float32 y\n\
float32 theta\n\
";
  }

  static const char* value(const ::dart_delivery::RosieOdom_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dart_delivery::RosieOdom_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RosieOdom_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dart_delivery::RosieOdom_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dart_delivery::RosieOdom_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DART_DELIVERY_MESSAGE_ROSIEODOM_H
