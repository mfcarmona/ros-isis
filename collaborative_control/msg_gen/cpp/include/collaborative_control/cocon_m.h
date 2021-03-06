/* Auto-generated by genmsg_cpp for file /home/distaur/.fakeHome/Dropbox/tmp/isis/collaborative_control/msg/cocon_m.msg */
#ifndef COLLABORATIVE_CONTROL_MESSAGE_COCON_M_H
#define COLLABORATIVE_CONTROL_MESSAGE_COCON_M_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "efficiency/Efficiency_m.h"
#include "geometry_msgs/Point.h"
#include "efficiency/Efficiency_m.h"
#include "geometry_msgs/Point.h"
#include "efficiency/Efficiency_m.h"

namespace collaborative_control
{
template <class ContainerAllocator>
struct cocon_m_ : public ros::Message
{
  typedef cocon_m_<ContainerAllocator> Type;

  cocon_m_()
  : robotPos()
  , obstacles()
  , target()
  , weigthA(0.0)
  , comandA()
  , etaA()
  , weigthB(0.0)
  , comandB()
  , etaB()
  , comandC()
  , etaC()
  {
  }

  cocon_m_(const ContainerAllocator& _alloc)
  : robotPos(_alloc)
  , obstacles(_alloc)
  , target(_alloc)
  , weigthA(0.0)
  , comandA(_alloc)
  , etaA(_alloc)
  , weigthB(0.0)
  , comandB(_alloc)
  , etaB(_alloc)
  , comandC(_alloc)
  , etaC(_alloc)
  {
  }

  typedef  ::nav_msgs::Odometry_<ContainerAllocator>  _robotPos_type;
   ::nav_msgs::Odometry_<ContainerAllocator>  robotPos;

  typedef  ::nav_msgs::GridCells_<ContainerAllocator>  _obstacles_type;
   ::nav_msgs::GridCells_<ContainerAllocator>  obstacles;

  typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _target_type;
   ::geometry_msgs::Pose2D_<ContainerAllocator>  target;

  typedef float _weigthA_type;
  float weigthA;

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _comandA_type;
   ::geometry_msgs::Point_<ContainerAllocator>  comandA;

  typedef  ::efficiency::Efficiency_m_<ContainerAllocator>  _etaA_type;
   ::efficiency::Efficiency_m_<ContainerAllocator>  etaA;

  typedef float _weigthB_type;
  float weigthB;

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _comandB_type;
   ::geometry_msgs::Point_<ContainerAllocator>  comandB;

  typedef  ::efficiency::Efficiency_m_<ContainerAllocator>  _etaB_type;
   ::efficiency::Efficiency_m_<ContainerAllocator>  etaB;

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _comandC_type;
   ::geometry_msgs::Point_<ContainerAllocator>  comandC;

  typedef  ::efficiency::Efficiency_m_<ContainerAllocator>  _etaC_type;
   ::efficiency::Efficiency_m_<ContainerAllocator>  etaC;


private:
  static const char* __s_getDataType_() { return "collaborative_control/cocon_m"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "f5422d830552858705e5ab3bb7b43b70"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "nav_msgs/Odometry robotPos\n\
nav_msgs/GridCells obstacles\n\
geometry_msgs/Pose2D target\n\
float32 weigthA\n\
geometry_msgs/Point comandA\n\
efficiency/Efficiency_m etaA\n\
float32 weigthB\n\
geometry_msgs/Point comandB\n\
efficiency/Efficiency_m etaB\n\
geometry_msgs/Point comandC\n\
efficiency/Efficiency_m etaC\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertianty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: nav_msgs/GridCells\n\
#an array of cells in a 2D grid\n\
Header header\n\
float32 cell_width\n\
float32 cell_height\n\
geometry_msgs/Point[] cells\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
================================================================================\n\
MSG: efficiency/Efficiency_m\n\
float32 Global\n\
float32 Safety\n\
float32 Directness\n\
float32 Smoothness\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, robotPos);
    ros::serialization::serialize(stream, obstacles);
    ros::serialization::serialize(stream, target);
    ros::serialization::serialize(stream, weigthA);
    ros::serialization::serialize(stream, comandA);
    ros::serialization::serialize(stream, etaA);
    ros::serialization::serialize(stream, weigthB);
    ros::serialization::serialize(stream, comandB);
    ros::serialization::serialize(stream, etaB);
    ros::serialization::serialize(stream, comandC);
    ros::serialization::serialize(stream, etaC);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, robotPos);
    ros::serialization::deserialize(stream, obstacles);
    ros::serialization::deserialize(stream, target);
    ros::serialization::deserialize(stream, weigthA);
    ros::serialization::deserialize(stream, comandA);
    ros::serialization::deserialize(stream, etaA);
    ros::serialization::deserialize(stream, weigthB);
    ros::serialization::deserialize(stream, comandB);
    ros::serialization::deserialize(stream, etaB);
    ros::serialization::deserialize(stream, comandC);
    ros::serialization::deserialize(stream, etaC);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(robotPos);
    size += ros::serialization::serializationLength(obstacles);
    size += ros::serialization::serializationLength(target);
    size += ros::serialization::serializationLength(weigthA);
    size += ros::serialization::serializationLength(comandA);
    size += ros::serialization::serializationLength(etaA);
    size += ros::serialization::serializationLength(weigthB);
    size += ros::serialization::serializationLength(comandB);
    size += ros::serialization::serializationLength(etaB);
    size += ros::serialization::serializationLength(comandC);
    size += ros::serialization::serializationLength(etaC);
    return size;
  }

  typedef boost::shared_ptr< ::collaborative_control::cocon_m_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::collaborative_control::cocon_m_<ContainerAllocator>  const> ConstPtr;
}; // struct cocon_m
typedef  ::collaborative_control::cocon_m_<std::allocator<void> > cocon_m;

typedef boost::shared_ptr< ::collaborative_control::cocon_m> cocon_mPtr;
typedef boost::shared_ptr< ::collaborative_control::cocon_m const> cocon_mConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::collaborative_control::cocon_m_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::collaborative_control::cocon_m_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace collaborative_control

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::collaborative_control::cocon_m_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f5422d830552858705e5ab3bb7b43b70";
  }

  static const char* value(const  ::collaborative_control::cocon_m_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf5422d8305528587ULL;
  static const uint64_t static_value2 = 0x05e5ab3bb7b43b70ULL;
};

template<class ContainerAllocator>
struct DataType< ::collaborative_control::cocon_m_<ContainerAllocator> > {
  static const char* value() 
  {
    return "collaborative_control/cocon_m";
  }

  static const char* value(const  ::collaborative_control::cocon_m_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::collaborative_control::cocon_m_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nav_msgs/Odometry robotPos\n\
nav_msgs/GridCells obstacles\n\
geometry_msgs/Pose2D target\n\
float32 weigthA\n\
geometry_msgs/Point comandA\n\
efficiency/Efficiency_m etaA\n\
float32 weigthB\n\
geometry_msgs/Point comandB\n\
efficiency/Efficiency_m etaB\n\
geometry_msgs/Point comandC\n\
efficiency/Efficiency_m etaC\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertianty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: nav_msgs/GridCells\n\
#an array of cells in a 2D grid\n\
Header header\n\
float32 cell_width\n\
float32 cell_height\n\
geometry_msgs/Point[] cells\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
================================================================================\n\
MSG: efficiency/Efficiency_m\n\
float32 Global\n\
float32 Safety\n\
float32 Directness\n\
float32 Smoothness\n\
\n\
\n\
";
  }

  static const char* value(const  ::collaborative_control::cocon_m_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::collaborative_control::cocon_m_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.robotPos);
    stream.next(m.obstacles);
    stream.next(m.target);
    stream.next(m.weigthA);
    stream.next(m.comandA);
    stream.next(m.etaA);
    stream.next(m.weigthB);
    stream.next(m.comandB);
    stream.next(m.etaB);
    stream.next(m.comandC);
    stream.next(m.etaC);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct cocon_m_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::collaborative_control::cocon_m_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::collaborative_control::cocon_m_<ContainerAllocator> & v) 
  {
    s << indent << "robotPos: ";
s << std::endl;
    Printer< ::nav_msgs::Odometry_<ContainerAllocator> >::stream(s, indent + "  ", v.robotPos);
    s << indent << "obstacles: ";
s << std::endl;
    Printer< ::nav_msgs::GridCells_<ContainerAllocator> >::stream(s, indent + "  ", v.obstacles);
    s << indent << "target: ";
s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.target);
    s << indent << "weigthA: ";
    Printer<float>::stream(s, indent + "  ", v.weigthA);
    s << indent << "comandA: ";
s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.comandA);
    s << indent << "etaA: ";
s << std::endl;
    Printer< ::efficiency::Efficiency_m_<ContainerAllocator> >::stream(s, indent + "  ", v.etaA);
    s << indent << "weigthB: ";
    Printer<float>::stream(s, indent + "  ", v.weigthB);
    s << indent << "comandB: ";
s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.comandB);
    s << indent << "etaB: ";
s << std::endl;
    Printer< ::efficiency::Efficiency_m_<ContainerAllocator> >::stream(s, indent + "  ", v.etaB);
    s << indent << "comandC: ";
s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.comandC);
    s << indent << "etaC: ";
s << std::endl;
    Printer< ::efficiency::Efficiency_m_<ContainerAllocator> >::stream(s, indent + "  ", v.etaC);
  }
};


} // namespace message_operations
} // namespace ros

#endif // COLLABORATIVE_CONTROL_MESSAGE_COCON_M_H

