// Generated by gencpp from file quad_controller/SetFloatRequest.msg
// DO NOT EDIT!


#ifndef QUAD_CONTROLLER_MESSAGE_SETFLOATREQUEST_H
#define QUAD_CONTROLLER_MESSAGE_SETFLOATREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace quad_controller
{
template <class ContainerAllocator>
struct SetFloatRequest_
{
  typedef SetFloatRequest_<ContainerAllocator> Type;

  SetFloatRequest_()
    : data(0.0)  {
    }
  SetFloatRequest_(const ContainerAllocator& _alloc)
    : data(0.0)  {
  (void)_alloc;
    }



   typedef float _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::quad_controller::SetFloatRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quad_controller::SetFloatRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetFloatRequest_

typedef ::quad_controller::SetFloatRequest_<std::allocator<void> > SetFloatRequest;

typedef boost::shared_ptr< ::quad_controller::SetFloatRequest > SetFloatRequestPtr;
typedef boost::shared_ptr< ::quad_controller::SetFloatRequest const> SetFloatRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quad_controller::SetFloatRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quad_controller::SetFloatRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace quad_controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'quad_controller': ['/media/juan/Datos/Repositorios/Udacity-Robo-nanodegree/ros-ws/5-catkin_ws/src/RoboND-Controls-Lab/quad_controller/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quad_controller::SetFloatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quad_controller::SetFloatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quad_controller::SetFloatRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "73fcbf46b49191e672908e50842a83d4";
  }

  static const char* value(const ::quad_controller::SetFloatRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x73fcbf46b49191e6ULL;
  static const uint64_t static_value2 = 0x72908e50842a83d4ULL;
};

template<class ContainerAllocator>
struct DataType< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quad_controller/SetFloatRequest";
  }

  static const char* value(const ::quad_controller::SetFloatRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 data\n\
";
  }

  static const char* value(const ::quad_controller::SetFloatRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetFloatRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quad_controller::SetFloatRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quad_controller::SetFloatRequest_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<float>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUAD_CONTROLLER_MESSAGE_SETFLOATREQUEST_H
