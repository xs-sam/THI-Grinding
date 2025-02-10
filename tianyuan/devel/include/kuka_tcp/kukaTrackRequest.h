// Generated by gencpp from file kuka_tcp/kukaTrackRequest.msg
// DO NOT EDIT!


#ifndef KUKA_TCP_MESSAGE_KUKATRACKREQUEST_H
#define KUKA_TCP_MESSAGE_KUKATRACKREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kuka_tcp/kukaPoint.h>

namespace kuka_tcp
{
template <class ContainerAllocator>
struct kukaTrackRequest_
{
  typedef kukaTrackRequest_<ContainerAllocator> Type;

  kukaTrackRequest_()
    : track()
    , speed(0)
    , mod(0)  {
    }
  kukaTrackRequest_(const ContainerAllocator& _alloc)
    : track(_alloc)
    , speed(0)
    , mod(0)  {
  (void)_alloc;
    }



   typedef std::vector< ::kuka_tcp::kukaPoint_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::kuka_tcp::kukaPoint_<ContainerAllocator> >> _track_type;
  _track_type track;

   typedef int32_t _speed_type;
  _speed_type speed;

   typedef int32_t _mod_type;
  _mod_type mod;





  typedef boost::shared_ptr< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> const> ConstPtr;

}; // struct kukaTrackRequest_

typedef ::kuka_tcp::kukaTrackRequest_<std::allocator<void> > kukaTrackRequest;

typedef boost::shared_ptr< ::kuka_tcp::kukaTrackRequest > kukaTrackRequestPtr;
typedef boost::shared_ptr< ::kuka_tcp::kukaTrackRequest const> kukaTrackRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator1> & lhs, const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator2> & rhs)
{
  return lhs.track == rhs.track &&
    lhs.speed == rhs.speed &&
    lhs.mod == rhs.mod;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator1> & lhs, const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kuka_tcp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f55ac92146a46389f08a770a8fc980c0";
  }

  static const char* value(const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf55ac92146a46389ULL;
  static const uint64_t static_value2 = 0xf08a770a8fc980c0ULL;
};

template<class ContainerAllocator>
struct DataType< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kuka_tcp/kukaTrackRequest";
  }

  static const char* value(const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kukaPoint[] track\n"
"int32 speed # mm/s\n"
"int32 mod # movel 1  movej 2\n"
"\n"
"================================================================================\n"
"MSG: kuka_tcp/kukaPoint\n"
"# 机器人点位信息\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 a\n"
"float32 b\n"
"float32 c\n"
;
  }

  static const char* value(const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.track);
      stream.next(m.speed);
      stream.next(m.mod);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct kukaTrackRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kuka_tcp::kukaTrackRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kuka_tcp::kukaTrackRequest_<ContainerAllocator>& v)
  {
    s << indent << "track[]" << std::endl;
    for (size_t i = 0; i < v.track.size(); ++i)
    {
      s << indent << "  track[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kuka_tcp::kukaPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.track[i]);
    }
    s << indent << "speed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.speed);
    s << indent << "mod: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mod);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KUKA_TCP_MESSAGE_KUKATRACKREQUEST_H
