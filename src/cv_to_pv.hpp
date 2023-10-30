#pragma once
#include <cstddef>
#include <libcamera/controls.h>
#include <rclcpp/parameter_value.hpp>


class invalid_conversion : public std::runtime_error
{
public:
  explicit invalid_conversion(const std::string &msg) : std::runtime_error(msg) {};
};


rclcpp::ParameterValue
cv_to_pv(const libcamera::ControlValue &value);

rclcpp::ParameterType
cv_to_pv_type(const libcamera::ControlType &type, const bool is_array);
