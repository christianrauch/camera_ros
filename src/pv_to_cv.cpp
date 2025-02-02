#include "pv_to_cv.hpp"
#include "array_string_utils.hpp"
#include "exceptions.hpp"
#include "libcamera_version_utils.hpp"
#include "types.hpp"
#include <cstdint>
#include <libcamera/base/span.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <vector>


#define CASE_CONVERT_INT(T)       \
  case libcamera::ControlType##T: \
    return ControlTypeMap<libcamera::ControlType##T>::type(parameter.as_int());

#define CASE_CONVERT_INT_ARRAY(T) \
  case libcamera::ControlType##T: \
    return libcamera::Span<const ControlTypeMap<libcamera::ControlType##T>::type>(std::vector<ControlTypeMap<libcamera::ControlType##T>::type>(values.begin(), values.end()));

#define CASE_CONVERT_STR(T)       \
  case libcamera::ControlType##T: \
    return CTString(parameter.as_string());

#define CASE_NONE(T)              \
  case libcamera::ControlType##T: \
    return {};

#define CASE_INVALID(T)           \
  case libcamera::ControlType##T: \
    throw invalid_conversion("cannot convert integer array to ##T");


libcamera::ControlValue
pv_to_cv_int_array(const std::vector<int64_t> &values, const libcamera::ControlType &type)
{
  // convert to integer Span, geometric type Rectangle, Size, Point, or throw exception
  switch (type) {
    CASE_NONE(None)
    CASE_INVALID(Bool)
    CASE_INVALID(Byte)
    CASE_CONVERT_INT_ARRAY(Integer32)
  case libcamera::ControlTypeInteger64:
    return libcamera::Span<const CTInteger64>(values);
#if LIBCAMERA_VER_GE(0, 4, 0)
    CASE_CONVERT_INT_ARRAY(Unsigned16)
    CASE_CONVERT_INT_ARRAY(Unsigned32)
#endif
    CASE_CONVERT_INT_ARRAY(Float)
    CASE_INVALID(String)
  case libcamera::ControlTypeRectangle:
    return libcamera::Rectangle(values[0], values[1], values[2], values[3]);
  case libcamera::ControlTypeSize:
    return libcamera::Size(values[0], values[1]);
#if LIBCAMERA_VER_GE(0, 4, 0)
  case libcamera::ControlTypePoint:
    return libcamera::Point(values[0], values[1]);
#endif
  }
  throw should_not_reach();
}

libcamera::ControlValue
pv_to_cv(const rclcpp::Parameter &parameter, const libcamera::ControlType &type)
{
  switch (parameter.get_type()) {
  case rclcpp::ParameterType::PARAMETER_NOT_SET:
    return {};
  case rclcpp::ParameterType::PARAMETER_BOOL:
    return parameter.as_bool();
  case rclcpp::ParameterType::PARAMETER_INTEGER:
    switch (type) {
      CASE_NONE(None)
      CASE_CONVERT_INT(Bool)
      CASE_CONVERT_INT(Byte)
      CASE_CONVERT_INT(Integer32)
      CASE_CONVERT_INT(Integer64)
      CASE_CONVERT_INT(Float)
      CASE_NONE(String)
      CASE_NONE(Rectangle)
      CASE_NONE(Size)
#if LIBCAMERA_VER_GE(0, 4, 0)
      CASE_CONVERT_INT(Unsigned16)
      CASE_CONVERT_INT(Unsigned32)
      CASE_NONE(Point)
#endif
    }
    throw should_not_reach();
  case rclcpp::ParameterType::PARAMETER_DOUBLE:
    return CTFloat(parameter.as_double());
  case rclcpp::ParameterType::PARAMETER_STRING:
    switch (type) {
      CASE_NONE(None)
      CASE_CONVERT_STR(Bool)
      CASE_CONVERT_STR(Byte)
      CASE_CONVERT_STR(Integer32)
      CASE_CONVERT_STR(Integer64)
      CASE_CONVERT_STR(Float)
      CASE_CONVERT_STR(String)
    case libcamera::ControlTypeRectangle:
    {
      std::vector<std::vector<int>> decoded_array = decode_2d_numeric_array<int>(parameter.as_string());
      std::vector<libcamera::Rectangle> decoded_array_converted;

      for (auto &item : decoded_array) {
        if (item.size() < 4)
          continue;

        decoded_array_converted.emplace_back(item[0], item[1], item[2], item[3]);
      }

      return libcamera::Span<const ControlTypeMap<libcamera::ControlTypeRectangle>::type>(decoded_array_converted);
    }
      CASE_CONVERT_STR(Size)
#if LIBCAMERA_VER_GE(0, 4, 0)
      CASE_CONVERT_STR(Unsigned16)
      CASE_CONVERT_STR(Unsigned32)
    case libcamera::ControlTypePoint:
    {
      std::vector<std::vector<int>> decoded_array = decode_2d_numeric_array<int>(parameter.as_string());
      std::vector<libcamera::Point> decoded_array_converted;

      for (auto &item : decoded_array) {
        if (item.size() < 2)
          continue;

        decoded_array_converted.emplace_back(item[0], item[1]);
      }

      return libcamera::Span<const ControlTypeMap<libcamera::ControlTypePoint>::type>(decoded_array_converted);
    }
#endif
    }
    throw should_not_reach();
  case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
    return libcamera::Span<const CTByte>(parameter.as_byte_array());
  case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    throw invalid_conversion("cannot convert bool array to control value");
  case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    return pv_to_cv_int_array(parameter.as_integer_array(), type);
  case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
    // convert to float vector
    return libcamera::Span<const CTFloat>(
      std::vector<CTFloat>(parameter.as_double_array().begin(), parameter.as_double_array().end()));
  case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
    return libcamera::Span<const CTString>(parameter.as_string_array());
  }
  throw should_not_reach();
}
