#pragma once
#include "parameter_conflict_check.hpp"
#include <libcamera/controls.h>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace rclcpp
{
class Node;
class Parameter;
} // namespace rclcpp


class ParameterHandler
{
public:
  typedef std::unordered_map<unsigned int, libcamera::ControlValue> ControlValueMap;

  ParameterHandler(rclcpp::Node *const node);

  void
  declareFromControls(const libcamera::ControlInfoMap &controls);

  std::tuple<ControlValueMap, std::vector<std::string>>
  parameterCheckAndConvert(const std::vector<rclcpp::Parameter> &parameters);

private:
  rclcpp::Node *const node;
  std::unordered_map<std::string, const libcamera::ControlId *> parameter_ids;
  std::unordered_map<std::string, libcamera::ControlInfo> parameter_info;
  // keep track of set parameters
  ParameterMap parameters_full;
};
