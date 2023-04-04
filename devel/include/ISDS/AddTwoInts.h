// Generated by gencpp from file ISDS/AddTwoInts.msg
// DO NOT EDIT!


#ifndef ISDS_MESSAGE_ADDTWOINTS_H
#define ISDS_MESSAGE_ADDTWOINTS_H

#include <ros/service_traits.h>


#include <ISDS/AddTwoIntsRequest.h>
#include <ISDS/AddTwoIntsResponse.h>


namespace ISDS
{

struct AddTwoInts
{

typedef AddTwoIntsRequest Request;
typedef AddTwoIntsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddTwoInts
} // namespace ISDS


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ISDS::AddTwoInts > {
  static const char* value()
  {
    return "6a2e34150c00229791cc89ff309fff21";
  }

  static const char* value(const ::ISDS::AddTwoInts&) { return value(); }
};

template<>
struct DataType< ::ISDS::AddTwoInts > {
  static const char* value()
  {
    return "ISDS/AddTwoInts";
  }

  static const char* value(const ::ISDS::AddTwoInts&) { return value(); }
};


// service_traits::MD5Sum< ::ISDS::AddTwoIntsRequest> should match
// service_traits::MD5Sum< ::ISDS::AddTwoInts >
template<>
struct MD5Sum< ::ISDS::AddTwoIntsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ISDS::AddTwoInts >::value();
  }
  static const char* value(const ::ISDS::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ISDS::AddTwoIntsRequest> should match
// service_traits::DataType< ::ISDS::AddTwoInts >
template<>
struct DataType< ::ISDS::AddTwoIntsRequest>
{
  static const char* value()
  {
    return DataType< ::ISDS::AddTwoInts >::value();
  }
  static const char* value(const ::ISDS::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ISDS::AddTwoIntsResponse> should match
// service_traits::MD5Sum< ::ISDS::AddTwoInts >
template<>
struct MD5Sum< ::ISDS::AddTwoIntsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ISDS::AddTwoInts >::value();
  }
  static const char* value(const ::ISDS::AddTwoIntsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ISDS::AddTwoIntsResponse> should match
// service_traits::DataType< ::ISDS::AddTwoInts >
template<>
struct DataType< ::ISDS::AddTwoIntsResponse>
{
  static const char* value()
  {
    return DataType< ::ISDS::AddTwoInts >::value();
  }
  static const char* value(const ::ISDS::AddTwoIntsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ISDS_MESSAGE_ADDTWOINTS_H
