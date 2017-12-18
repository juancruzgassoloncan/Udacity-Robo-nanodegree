// Generated by gencpp from file quad_controller/SetPose.msg
// DO NOT EDIT!


#ifndef QUAD_CONTROLLER_MESSAGE_SETPOSE_H
#define QUAD_CONTROLLER_MESSAGE_SETPOSE_H

#include <ros/service_traits.h>


#include <quad_controller/SetPoseRequest.h>
#include <quad_controller/SetPoseResponse.h>


namespace quad_controller
{

struct SetPose
{

typedef SetPoseRequest Request;
typedef SetPoseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetPose
} // namespace quad_controller


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::quad_controller::SetPose > {
  static const char* value()
  {
    return "28e4dd667b29bd35b516ba1d379b7529";
  }

  static const char* value(const ::quad_controller::SetPose&) { return value(); }
};

template<>
struct DataType< ::quad_controller::SetPose > {
  static const char* value()
  {
    return "quad_controller/SetPose";
  }

  static const char* value(const ::quad_controller::SetPose&) { return value(); }
};


// service_traits::MD5Sum< ::quad_controller::SetPoseRequest> should match 
// service_traits::MD5Sum< ::quad_controller::SetPose > 
template<>
struct MD5Sum< ::quad_controller::SetPoseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::quad_controller::SetPose >::value();
  }
  static const char* value(const ::quad_controller::SetPoseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::quad_controller::SetPoseRequest> should match 
// service_traits::DataType< ::quad_controller::SetPose > 
template<>
struct DataType< ::quad_controller::SetPoseRequest>
{
  static const char* value()
  {
    return DataType< ::quad_controller::SetPose >::value();
  }
  static const char* value(const ::quad_controller::SetPoseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::quad_controller::SetPoseResponse> should match 
// service_traits::MD5Sum< ::quad_controller::SetPose > 
template<>
struct MD5Sum< ::quad_controller::SetPoseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::quad_controller::SetPose >::value();
  }
  static const char* value(const ::quad_controller::SetPoseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::quad_controller::SetPoseResponse> should match 
// service_traits::DataType< ::quad_controller::SetPose > 
template<>
struct DataType< ::quad_controller::SetPoseResponse>
{
  static const char* value()
  {
    return DataType< ::quad_controller::SetPose >::value();
  }
  static const char* value(const ::quad_controller::SetPoseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QUAD_CONTROLLER_MESSAGE_SETPOSE_H
