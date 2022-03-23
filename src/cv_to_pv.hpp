#pragma once
#include <libcamera/controls.h>
#include <rclcpp/parameter_value.hpp>


rclcpp::ParameterValue cv_to_pv(const libcamera::ControlValue &value);
