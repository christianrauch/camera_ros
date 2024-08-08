#pragma once
// #include <libcamera/camera.h>
#include <libcamera/controls.h>
#include <rclcpp/node.hpp>


class ParameterHandler
{
public:
  ParameterHandler(rclcpp::Node *const node);

  void
  declare(const libcamera::ControlInfoMap &controls);

private:
  rclcpp::Node *const node;
  std::unordered_map<std::string, const libcamera::ControlId *> parameter_ids;
};
