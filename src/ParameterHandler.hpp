#pragma once
#include "ParameterConflictHandler.hpp"
#include <functional>
#include <libcamera/controls.h>
#include <mutex>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>


namespace rclcpp
{
class Node;
} // namespace rclcpp


class ParameterHandler
{
public:
  explicit ParameterHandler(rclcpp::Node *const node);

  void
  declare(const libcamera::ControlInfoMap &controls);

  const libcamera::ControlList &
  get_control_values();

  void
  move_control_values(libcamera::ControlList &controls);

  void
  redeclare();

private:
  struct control_info_t
  {
    const libcamera::ControlId *id;
    libcamera::ControlInfo info;
  };

  rclcpp::Node *const node;
  ParameterConflictHandler parameter_conflict_handler;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_on;
#ifdef RCLCPP_HAS_PARAM_EXT_CB
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr param_cb_pre;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_post;
#endif

  std::unordered_map<std::string, const control_info_t> camera_controls;

  libcamera::ControlList control_values;
  std::mutex control_values_lock;

  std::unordered_map<std::string, rcl_interfaces::msg::ParameterDescriptor> parameter_descriptors;

  void
  PreSetResolve(std::vector<rclcpp::Parameter> &parameters);

  rcl_interfaces::msg::SetParametersResult
  OnSetValidate(const std::vector<rclcpp::Parameter> &parameters);

  void
  PostSetApply(const std::vector<rclcpp::Parameter> &parameters);

  std::vector<std::string>
  validate_new_parameters(const std::vector<rclcpp::Parameter> &parameters);
};
