#pragma once
#include <libcamera/controls.h>
#include <rclcpp/parameter_value.hpp>


rclcpp::ParameterValue
cv_to_pv(const libcamera::ControlValue &value, const std::size_t &extent);

rclcpp::ParameterType
cv_to_pv_type(const libcamera::ControlType &type, const bool is_array);
