// Generated by gencpp from file thi_vision/orientation.msg
// DO NOT EDIT!


#ifndef THI_VISION_MESSAGE_ORIENTATION_H
#define THI_VISION_MESSAGE_ORIENTATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace thi_vision
{
template <class ContainerAllocator>
struct orientation_
{
  typedef orientation_<ContainerAllocator> Type;

  orientation_()
    : a(0.0)
    , b(0.0)
    , c(0.0)  {
    }
  orientation_(const ContainerAllocator& _alloc)
    : a(0.0)
    , b(0.0)
    , c(0.0)  {
  (void)_alloc;
    }



   typedef float _a_type;
  _a_type a;

   typedef float _b_type;
  _b_type b;

   typedef float _c_type;
  _c_type c;





  typedef boost::shared_ptr< ::thi_vision::orientation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::thi_vision::orientation_<ContainerAllocator> const> ConstPtr;

}; // struct orientation_

typedef ::thi_vision::orientation_<std::allocator<void> > orientation;

typedef boost::shared_ptr< ::thi_vision::orientation > orientationPtr;
typedef boost::shared_ptr< ::thi_vision::orientation const> orientationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::thi_vision::orientation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::thi_vision::orientation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::thi_vision::orientation_<ContainerAllocator1> & lhs, const ::thi_vision::orientation_<ContainerAllocator2> & rhs)
{
  return lhs.a == rhs.a &&
    lhs.b == rhs.b &&
    lhs.c == rhs.c;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::thi_vision::orientation_<ContainerAllocator1> & lhs, const ::thi_vision::orientation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace thi_vision

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::thi_vision::orientation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::thi_vision::orientation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::thi_vision::orientation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::thi_vision::orientation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::thi_vision::orientation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::thi_vision::orientation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::thi_vision::orientation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d20f63a7e99cd5689c2dcf93cf2f8085";
  }

  static const char* value(const ::thi_vision::orientation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd20f63a7e99cd568ULL;
  static const uint64_t static_value2 = 0x9c2dcf93cf2f8085ULL;
};

template<class ContainerAllocator>
struct DataType< ::thi_vision::orientation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "thi_vision/orientation";
  }

  static const char* value(const ::thi_vision::orientation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::thi_vision::orientation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 a\n"
"float32 b\n"
"float32 c\n"
;
  }

  static const char* value(const ::thi_vision::orientation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::thi_vision::orientation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
      stream.next(m.c);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct orientation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::thi_vision::orientation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::thi_vision::orientation_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<float>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<float>::stream(s, indent + "  ", v.b);
    s << indent << "c: ";
    Printer<float>::stream(s, indent + "  ", v.c);
  }
};

} // namespace message_operations
} // namespace ros

#endif // THI_VISION_MESSAGE_ORIENTATION_H
