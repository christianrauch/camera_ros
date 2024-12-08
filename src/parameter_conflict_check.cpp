#include "parameter_conflict_check.hpp"
#include <algorithm>
#include <rclcpp/parameter.hpp>


std::tuple<ParameterMap, std::vector<std::string>>
resolve_conflicts(const ParameterMap &parameters_default, const ParameterMap &parameters_overrides)
{
  ParameterMap parameters_init = parameters_default;
  std::vector<std::string> msgs;

  // auto exposure (AeEnable) and manual exposure (ExposureTime)
  // must not be enabled at the same time

  // default: prefer auto exposure
  if (parameters_init.count("AeEnable") &&
      (parameters_init.at("AeEnable").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) &&
      parameters_init.at("AeEnable").get<bool>() &&
      parameters_init.count("ExposureTime"))
  {
    // disable exposure
    parameters_init.erase("ExposureTime");
  }

  // apply parameter overrides
  for (const auto &[name, value] : parameters_overrides) {
    // only override parameters that have matching controls
    if (parameters_default.count(name))
      parameters_init[name] = value;
  }

  // overrides: prefer provided exposure
  if (parameters_init.count("AeEnable") &&
      (parameters_init.at("AeEnable").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) &&
      parameters_init.at("AeEnable").get<bool>() &&
      parameters_init.count("ExposureTime"))
  {
    // disable auto exposure
    parameters_init.at("AeEnable") = rclcpp::ParameterValue(false);
    msgs.emplace_back("AeEnable and ExposureTime must not be enabled at the same time. 'AeEnable' "
                      "will be set to off.");
  }

  return {parameters_init, msgs};
}

std::vector<std::string>
check_conflicts(const std::vector<rclcpp::Parameter> &parameters_new,
                const ParameterMap &parameters_full)
{
  std::vector<std::string> msgs;

  ParameterMap parameter_map;
  // old configuration state
  for (const auto &[name, value] : parameters_full)
    parameter_map[name] = value;
  // apply new configuration update
  for (const auto &p : parameters_new)
    parameter_map[p.get_name()] = p.get_parameter_value();

  // is auto exposure going to be enabled?
  const bool ae_enabled =
    parameter_map.count("AeEnable") &&
    (parameter_map.at("AeEnable").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) &&
    parameter_map.at("AeEnable").get<bool>();
  // are new parameters setting the exposure manually?
  const bool exposure_updated =
    std::find_if(parameters_new.begin(), parameters_new.end(), [](const rclcpp::Parameter &param) {
      return param.get_name() == "ExposureTime";
    }) != parameters_new.end();

  // ExposureTime must not be set while AeEnable is true
  if (ae_enabled && exposure_updated)
    msgs.emplace_back("AeEnable and ExposureTime must not be set simultaneously");

  return msgs;
}
