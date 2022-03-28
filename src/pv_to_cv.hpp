#pragma once
#include <libcamera/controls.h>
#include <rclcpp/parameter.hpp>


libcamera::ControlValue
pv_to_cv(const rclcpp::Parameter &parameter, const libcamera::ControlType &type);
