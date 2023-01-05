#pragma once
#include <map>
#include <rclcpp/parameter_value.hpp>
#include <string>
#include <tuple>
#include <vector>


namespace rclcpp
{
class Parameter;
}

typedef std::map<std::string, rclcpp::ParameterValue> ParameterMap;

std::tuple<ParameterMap, std::vector<std::string>>
resolve_conflicts(const ParameterMap &parameters_default, const ParameterMap &parameters_overrides);

std::vector<std::string>
check_conflicts(const std::vector<rclcpp::Parameter> &parameters_new,
                const ParameterMap &parameters_full);
