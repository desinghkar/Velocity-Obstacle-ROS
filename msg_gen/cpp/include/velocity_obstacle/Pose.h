/* Auto-generated by genmsg_cpp for file /workspace/karthik/RRC/ros_workspace/velocity_obstacle/msg/Pose.msg */
#ifndef VELOCITY_OBSTACLE_MESSAGE_POSE_H
#define VELOCITY_OBSTACLE_MESSAGE_POSE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace velocity_obstacle
{
template <class ContainerAllocator>
struct Pose_ {
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
  : x(0.0)
  , y(0.0)
  , theta(0.0)
  , linear_velocity(0.0)
  , angular_velocity(0.0)
  {
  }

  Pose_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , theta(0.0)
  , linear_velocity(0.0)
  , angular_velocity(0.0)
  {
  }

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _theta_type;
  float theta;

  typedef float _linear_velocity_type;
  float linear_velocity;

  typedef float _angular_velocity_type;
  float angular_velocity;


private:
  static const char* __s_getDataType_() { return "velocity_obstacle/Pose"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "863b248d5016ca62ea2e895ae5265cf9"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 x\n\
float32 y\n\
float32 theta\n\
\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    ros::serialization::serialize(stream, theta);
    ros::serialization::serialize(stream, linear_velocity);
    ros::serialization::serialize(stream, angular_velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, theta);
    ros::serialization::deserialize(stream, linear_velocity);
    ros::serialization::deserialize(stream, angular_velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(theta);
    size += ros::serialization::serializationLength(linear_velocity);
    size += ros::serialization::serializationLength(angular_velocity);
    return size;
  }

  typedef boost::shared_ptr< ::velocity_obstacle::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::velocity_obstacle::Pose_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Pose
typedef  ::velocity_obstacle::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::velocity_obstacle::Pose> PosePtr;
typedef boost::shared_ptr< ::velocity_obstacle::Pose const> PoseConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::velocity_obstacle::Pose_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::velocity_obstacle::Pose_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace velocity_obstacle

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::velocity_obstacle::Pose_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::velocity_obstacle::Pose_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::velocity_obstacle::Pose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "863b248d5016ca62ea2e895ae5265cf9";
  }

  static const char* value(const  ::velocity_obstacle::Pose_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x863b248d5016ca62ULL;
  static const uint64_t static_value2 = 0xea2e895ae5265cf9ULL;
};

template<class ContainerAllocator>
struct DataType< ::velocity_obstacle::Pose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "velocity_obstacle/Pose";
  }

  static const char* value(const  ::velocity_obstacle::Pose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::velocity_obstacle::Pose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 x\n\
float32 y\n\
float32 theta\n\
\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
";
  }

  static const char* value(const  ::velocity_obstacle::Pose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::velocity_obstacle::Pose_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::velocity_obstacle::Pose_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.theta);
    stream.next(m.linear_velocity);
    stream.next(m.angular_velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Pose_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::velocity_obstacle::Pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::velocity_obstacle::Pose_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
    s << indent << "linear_velocity: ";
    Printer<float>::stream(s, indent + "  ", v.linear_velocity);
    s << indent << "angular_velocity: ";
    Printer<float>::stream(s, indent + "  ", v.angular_velocity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // VELOCITY_OBSTACLE_MESSAGE_POSE_H

