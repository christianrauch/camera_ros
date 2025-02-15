#pragma once
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <string>
#include <unordered_map>
#include <vector>


class ParameterConflictHandler
{
public:
  typedef std::unordered_map<std::string, rclcpp::ParameterValue> ParameterValueMap;

  ParameterConflictHandler();

  std::vector<std::string>
  resolve_defaults(ParameterValueMap &p);

  std::vector<std::string>
  resolve_overrides(ParameterValueMap &p);

  void
  restore(std::vector<rclcpp::Parameter> &parameters);

  static std::vector<std::string>
  check(const std::vector<rclcpp::Parameter> &parameters_old,
        const std::vector<rclcpp::Parameter> &parameters_new);

  void
  store_commit_or_revert(const bool commit);

private:
  ParameterValueMap store;
  ParameterValueMap tmp_store;

  static bool
  conflict_exposure(const ParameterValueMap &p);
};
