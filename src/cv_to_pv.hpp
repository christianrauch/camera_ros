#pragma once
#include <cstddef>
#include <libcamera/controls.h>
#include <rclcpp/parameter_value.hpp>


rclcpp::ParameterValue
cv_to_pv(const libcamera::ControlValue &value);

rclcpp::ParameterType
cv_to_pv_type(const libcamera::ControlType &type, const bool is_array);
