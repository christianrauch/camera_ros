#pragma once
#include "types.hpp"
#include <libcamera/controls.h>
#include <rclcpp/parameter.hpp>

//#define CASE_CONVERT(T)                                                                            \
//  case rclcpp::ParameterType::PARAMETER_##T:                                                       \
//    return {parameter.get_value<rclcpp::ParameterType::PARAMETER_##T>()};

//#define CASE_NONE(T)                                                                               \
//  case rclcpp::ParameterType::PARAMETER_##T:                                                       \
//    return {};

//template<typename F, typename T, std::enable_if_t<std::is_same<F, T>::value, bool> = true>
//libcamera::Span<const T> vector_to_span(const std::vector<F> &values)
//{
//  return libcamera::Span<const T>(values);
//}

//template<typename F, typename T, std::enable_if_t<!std::is_same<F, T>::value, bool> = true>
//libcamera::Span<const T> vector_to_span(const std::vector<F> &values)
//{
//  return libcamera::Span<const T>(std::vector<T>(values.begin(), values.end()));
//}

libcamera::ControlValue pv_to_cv_int_array(const std::vector<int64_t> &values,
                                           const libcamera::ControlType &type)
{
  // convert to Span (Integer32, Integer64) or geometric type Rectangle, Size
  switch (type) {
  case libcamera::ControlTypeInteger32:
    return {
      libcamera::Span<const CTInteger32>(std::vector<CTInteger32>(values.begin(), values.end()))};
  case libcamera::ControlTypeInteger64:
    return {libcamera::Span<const CTInteger64>(values)};
  case libcamera::ControlTypeRectangle:
    return {libcamera::Rectangle(values[0], values[1], values[2], values[3])};
  case libcamera::ControlTypeSize:
    return {libcamera::Size(values[0], values[1])};
  default:
    return {};
  }

  return {};
}

libcamera::ControlValue pv_to_cv(const rclcpp::Parameter &parameter,
                                 const libcamera::ControlType &type)
{
  //  const std::string &name = parameter.get_name();

  //  switch (parameter.get_type()) {
  //    CASE_NONE(NOT_SET)
  //    CASE_CONVERT(BOOL)
  //    CASE_CONVERT(INTEGER)
  //    CASE_CONVERT(DOUBLE)
  //    CASE_CONVERT(STRING)
  //    CASE_CONVERT(BYTE_ARRAY)
  //    CASE_CONVERT(BOOL_ARRAY)
  //    CASE_CONVERT(INTEGER_ARRAY)
  //    CASE_CONVERT(DOUBLE_ARRAY)
  //    CASE_CONVERT(STRING_ARRAY)
  //  }

  switch (parameter.get_type()) {
  case rclcpp::ParameterType::PARAMETER_NOT_SET:
    return {};
  case rclcpp::ParameterType::PARAMETER_BOOL:
    return {parameter.as_bool()};
  case rclcpp::ParameterType::PARAMETER_INTEGER:
    if (type == libcamera::ControlTypeInteger32)
      return {CTInteger32(parameter.as_int())};
    else if (type == libcamera::ControlTypeInteger64)
      return {CTInteger64(parameter.as_int())};
    else
      return {};
  case rclcpp::ParameterType::PARAMETER_DOUBLE:
    return {CTFloat(parameter.as_double())};
  case rclcpp::ParameterType::PARAMETER_STRING:
    return {parameter.as_string()};
  case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
    return {libcamera::Span<const CTByte>(parameter.as_byte_array())};
  case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    return {};
  case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    return pv_to_cv_int_array(parameter.as_integer_array(), type);
  case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
  {
    // convert to float vector
    return {libcamera::Span<const CTFloat>(std::vector<CTFloat>(
      parameter.as_double_array().begin(), parameter.as_double_array().end()))};
  }
  case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
    return {libcamera::Span<const CTString>(parameter.as_string_array())};
  }

  return {};
}
