// Generated by gencpp from file data_handler/RegisterHWNodeRequest.msg
// DO NOT EDIT!


#ifndef DATA_HANDLER_MESSAGE_REGISTERHWNODEREQUEST_H
#define DATA_HANDLER_MESSAGE_REGISTERHWNODEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace data_handler
{
template <class ContainerAllocator>
struct RegisterHWNodeRequest_
{
  typedef RegisterHWNodeRequest_<ContainerAllocator> Type;

  RegisterHWNodeRequest_()
    : name()
    , topic()
    , hz(0)
    , use_dynamic_freq(false)  {
    }
  RegisterHWNodeRequest_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , topic(_alloc)
    , hz(0)
    , use_dynamic_freq(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topic_type;
  _topic_type topic;

   typedef uint64_t _hz_type;
  _hz_type hz;

   typedef uint8_t _use_dynamic_freq_type;
  _use_dynamic_freq_type use_dynamic_freq;





  typedef boost::shared_ptr< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RegisterHWNodeRequest_

typedef ::data_handler::RegisterHWNodeRequest_<std::allocator<void> > RegisterHWNodeRequest;

typedef boost::shared_ptr< ::data_handler::RegisterHWNodeRequest > RegisterHWNodeRequestPtr;
typedef boost::shared_ptr< ::data_handler::RegisterHWNodeRequest const> RegisterHWNodeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator1> & lhs, const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name &&
    lhs.topic == rhs.topic &&
    lhs.hz == rhs.hz &&
    lhs.use_dynamic_freq == rhs.use_dynamic_freq;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator1> & lhs, const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace data_handler

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "723425ee444bdb3453d240cc268418f0";
  }

  static const char* value(const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x723425ee444bdb34ULL;
  static const uint64_t static_value2 = 0x53d240cc268418f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "data_handler/RegisterHWNodeRequest";
  }

  static const char* value(const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
"string topic\n"
"uint64 hz\n"
"bool use_dynamic_freq\n"
;
  }

  static const char* value(const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.topic);
      stream.next(m.hz);
      stream.next(m.use_dynamic_freq);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RegisterHWNodeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::data_handler::RegisterHWNodeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::data_handler::RegisterHWNodeRequest_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "topic: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topic);
    s << indent << "hz: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.hz);
    s << indent << "use_dynamic_freq: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_dynamic_freq);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DATA_HANDLER_MESSAGE_REGISTERHWNODEREQUEST_H
