#pragma once
//#include "libcamera/controls.h"
#include "casts.hpp"
//#include <any>
//#include <libcamera/geometry.h>

//typedef std::vector<std::any> vec_any;

//vec_any cast_type(const libcamera::ControlValue &value_source)
//{
//  if (value_source.isArray()) {
//    // array
//    switch (value_source.type()) {
//    case libcamera::ControlTypeNone:
//      return {};
//    case libcamera::ControlTypeBool:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTBool>>();
//      return vec_any(v.begin(), v.end());
//    }
//      //      return value_source.get<libcamera::Span<const CTBool>>();
//    case libcamera::ControlTypeByte:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTByte>>();
//      return vec_any(v.begin(), v.end());
//    }
//    case libcamera::ControlTypeInteger32:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTInteger32>>();
//      return vec_any(v.begin(), v.end());
//    }
//    case libcamera::ControlTypeInteger64:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTInteger64>>();
//      return vec_any(v.begin(), v.end());
//    }
//    case libcamera::ControlTypeFloat:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTFloat>>();
//      return vec_any(v.begin(), v.end());
//    }
//    case libcamera::ControlTypeString:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTString>>();
//      return vec_any(v.begin(), v.end());
//    }
//    case libcamera::ControlTypeRectangle:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTRectangle>>();
//      return vec_any(v.begin(), v.end());
//    }
//    case libcamera::ControlTypeSize:
//    {
//      const auto v = value_source.get<libcamera::Span<const CTSize>>();
//      return vec_any(v.begin(), v.end());
//    }
//    }
//  }
//  else {
//    // scalar
//    switch (value_source.type()) {
//    case libcamera::ControlTypeNone:
//      return {};
//    case libcamera::ControlTypeBool:
//      return {value_source.get<CTBool>()};
//    case libcamera::ControlTypeByte:
//      return {value_source.get<CTByte>()};
//    case libcamera::ControlTypeInteger32:
//      return {value_source.get<CTInteger32>()};
//    case libcamera::ControlTypeInteger64:
//      return {value_source.get<CTInteger64>()};
//    case libcamera::ControlTypeFloat:
//      return {value_source.get<CTFloat>()};
//    case libcamera::ControlTypeString:
//      return {value_source.get<CTString>()};
//    case libcamera::ControlTypeRectangle:
//      return {value_source.get<CTRectangle>()};
//    case libcamera::ControlTypeSize:
//      return {value_source.get<CTSize>()};
//    }
//  }
//  return {};
//}


template<typename F, typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value &&
                            !std::is_same<std::remove_cv_t<T>, CTBool>::value &&
                            !(std::is_same<std::remove_cv_t<F>, CTRectangle>::value ||
                              std::is_same<std::remove_cv_t<T>, CTRectangle>::value ||
                              std::is_same<std::remove_cv_t<F>, CTSize>::value ||
                              std::is_same<std::remove_cv_t<T>, CTSize>::value),
                          bool> = true>
libcamera::ControlValue cast_cv(const libcamera::ControlValue &value)
{
  if (value.isArray()) {
    std::vector<T> a;
    for (const F &v : value.get<libcamera::Span<const F>>())
      a.push_back(cast<T, F>(v));
    return libcamera::ControlValue(libcamera::Span<const T>(a));
  }
  else {
    return libcamera::ControlValue(cast<T, F>(value.get<F>()));
  }
}

template<typename F, typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value &&
                            std::is_same<std::remove_cv_t<T>, CTBool>::value &&
                            !(std::is_same<std::remove_cv_t<F>, CTRectangle>::value ||
                              std::is_same<std::remove_cv_t<F>, CTSize>::value),
                          bool> = true>
libcamera::ControlValue cast_cv(const libcamera::ControlValue &value)
{
  if (value.isArray())
    throw std::runtime_error("unsupported array conversion for CTBool");
  else
    return libcamera::ControlValue(cast<T, F>(value.get<F>()));
}

template<
  typename F, typename T,
  std::enable_if_t<std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value, bool> = true>
const libcamera::ControlValue &cast_cv(const libcamera::ControlValue &value)
{
  return value;
}

template<typename F, typename T,
         std::enable_if_t<!std::is_same<std::remove_cv_t<F>, std::remove_cv_t<T>>::value &&
                            (std::is_same<std::remove_cv_t<F>, CTRectangle>::value ||
                             std::is_same<std::remove_cv_t<T>, CTRectangle>::value ||
                             std::is_same<std::remove_cv_t<F>, CTSize>::value ||
                             std::is_same<std::remove_cv_t<T>, CTSize>::value),
                          bool> = true>
libcamera::ControlValue cast_cv(const libcamera::ControlValue & /*value*/)
{
  throw std::runtime_error("unsupported ControlValue cast");
}

#define CASE_CAST_CONV1(F)                                                                         \
  case libcamera::ControlType##F:                                                                  \
    return cast_cv<ControlTypeMap<libcamera::ControlType##F>::type>(value);

#define CASE_CAST_CONV2(F)                                                                         \
  case libcamera::ControlType##F:                                                                  \
    return cast_cv<ControlTypeMap<libcamera::ControlType##F>::type, T>(value);

#define CASE_NONE(T)                                                                               \
  case libcamera::ControlType##T:                                                                  \
    return {};

template<typename T>
libcamera::ControlValue cast_cv(const libcamera::ControlValue &value)
{
  switch (value.type()) {
    CASE_NONE(None)
    CASE_CAST_CONV2(Bool)
    CASE_CAST_CONV2(Byte)
    CASE_CAST_CONV2(Integer32)
    CASE_CAST_CONV2(Integer64)
    CASE_CAST_CONV2(Float)
    CASE_CAST_CONV2(String)
    CASE_CAST_CONV2(Rectangle)
    CASE_CAST_CONV2(Size)
  }

  return {};
}

libcamera::ControlValue cast_cv(const libcamera::ControlValue &value,
                                const libcamera::ControlType target_type)
{
  switch (target_type) {
    CASE_NONE(None)
    CASE_CAST_CONV1(Bool)
    CASE_CAST_CONV1(Byte)
    CASE_CAST_CONV1(Integer32)
    CASE_CAST_CONV1(Integer64)
    CASE_CAST_CONV1(Float)
    CASE_CAST_CONV1(String)
    CASE_CAST_CONV1(Rectangle)
    CASE_CAST_CONV1(Size)
  }

  return {};
}

//template<typename T, typename F>
//T cast_any(const std::any &value)
//{
//  return cast<T, F>(std::any_cast<F>(value));
//}

//template<typename T>
//T cast_type(const std::any &value)
//{
//  //  if (typeid(T) == value.type())
//  //    return value;

//  if (value.type() == typeid(CTNone))
//    return {};
//  else if (value.type() == typeid(CTBool))
//    return cast_any<T, CTBool>(value);
//  else if (value.type() == typeid(CTByte))
//    return cast_any<T, CTByte>(value);
//  else if (value.type() == typeid(CTInteger32))
//    return cast_any<T, CTInteger32>(value);
//  else if (value.type() == typeid(CTInteger64))
//    return cast_any<T, CTInteger64>(value);
//  else if (value.type() == typeid(CTFloat))
//    return cast_any<T, CTFloat>(value);
//  else if (value.type() == typeid(CTString))
//    return cast_any<T, CTString>(value);
//  else if (value.type() == typeid(CTRectangle))
//    return {};
//  else if (value.type() == typeid(CTSize))
//    return {};
//  else
//    return {};

//  //  return value;
//}
