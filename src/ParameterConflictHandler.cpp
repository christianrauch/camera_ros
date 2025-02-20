#include "ParameterConflictHandler.hpp"
#include <libcamera/control_ids.h>


// use alias for control names from controls to avoid typos and detect API breakage
static const std::string &AE = libcamera::controls::AeEnable.name();
static const std::string &ET = libcamera::controls::ExposureTime.name();


auto
find(const std::vector<rclcpp::Parameter> &parameters,
     const std::string &name)
{
  return std::find_if(parameters.begin(), parameters.end(), [&name](const rclcpp::Parameter &parameter) {
    return (parameter.get_name() == name) && (parameter.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET);
  });
}

bool
is_set(const std::vector<rclcpp::Parameter> &parameters,
       const std::string &name)
{
  return find(parameters, name) != parameters.end();
}

bool
is_true(const std::vector<rclcpp::Parameter> &parameters,
        const std::string &name)
{
  const auto control = find(parameters, name);
  return (control != parameters.end()) &&
         (control->get_type() == rclcpp::ParameterType::PARAMETER_BOOL) &&
         control->as_bool();
}

std::vector<rclcpp::Parameter>
merge(const std::vector<rclcpp::Parameter> &param_old,
      const std::vector<rclcpp::Parameter> &param_new)
{
  // merge "new" parameters with "old" parameters, such that:
  //  1. the merged list is a superset of "old" and "new"
  //  2. "new" values replace "old" values for matching parameter names

  // initialise merged set with "new" parameters
  std::vector<rclcpp::Parameter> param_merged = param_new;
  // add "old" parameters to merged set, only if an "old" parameter is not overridden by any "new" parameter
  for (const rclcpp::Parameter &p : param_old) {
    if (!is_set(param_new, p.get_name())) {
      param_merged.emplace_back(p);
    }
  }
  return param_merged;
}

bool
is_set(const ParameterConflictHandler::ParameterValueMap &p,
       const std::string &name)
{
  return p.count(name) && (p.at(name).get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET);
}

ParameterConflictHandler::ParameterConflictHandler()
{
}

std::vector<std::string>
ParameterConflictHandler::resolve_defaults(ParameterValueMap &p)
{
  std::vector<std::string> msgs;

  // assume enabled for unset auto exposure (AeEnable)
  if (p.count(AE) && (p.at(AE).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)) {
    p.at(AE) = rclcpp::ParameterValue {true};
  }

  // default: prefer auto exposure
  if (conflict_exposure(p)) {
    // disable exposure
    store[ET] = p.at(ET);
    p.at(ET) = {};
    msgs.emplace_back("AeEnable and ExposureTime must not be enabled at the same time. 'ExposureTime' will be disabled.");
  }

  return msgs;
}

std::vector<std::string>
ParameterConflictHandler::resolve_overrides(ParameterValueMap &p)
{
  std::vector<std::string> msgs;

  // overrides: prefer provided exposure
  if (conflict_exposure(p)) {
    // disable auto exposure
    p.at(AE) = rclcpp::ParameterValue {false};
    msgs.emplace_back("AeEnable and ExposureTime must not be enabled at the same time. 'AeEnable' will be set to off.");
  }
  // restore 'ExposureTime'
  if (is_set(p, AE) && !p.at(AE).get<bool>()) {
    p.at(ET) = store.at(ET);
    store.erase(ET);
  }
  return msgs;
}

std::vector<std::string>
ParameterConflictHandler::check(const std::vector<rclcpp::Parameter> &parameters_old,
                                const std::vector<rclcpp::Parameter> &parameters_new)
{
  std::vector<std::string> msgs;

  const std::vector<rclcpp::Parameter> parameter_merged = merge(parameters_old, parameters_new);

  // ExposureTime must not be set while AeEnable is true
  if (is_set(parameter_merged, AE) && is_true(parameter_merged, AE) && is_set(parameters_new, ET))
    msgs.emplace_back("AeEnable and ExposureTime must not be set simultaneously");

  return msgs;
}

void
ParameterConflictHandler::restore(std::vector<rclcpp::Parameter> &parameters)
{
  // Temporarily store the restored parameter values and only apply them
  // if consecutive checks pass. If checks fail, drop these temporary
  // changes and keep the original store.

  assert(tmp_store.empty());

  tmp_store = store;

  if (is_set(parameters, AE)) {
    // restore 'ExposureTime' when 'AeEnable' is off
    if (!is_true(parameters, AE)) {
      if (tmp_store.count(ET)) {
        parameters.push_back({ET, tmp_store.at(ET)});
        tmp_store.erase(ET);
      }
    }
    else {
      parameters.push_back({ET, {}});
    }
  }
}

void
ParameterConflictHandler::store_commit_or_revert(const bool commit)
{
  if (commit) {
    // apply temporary changes
    store = std::move(tmp_store);
  }
  else {
    // remove temporary changes
    tmp_store.clear();
  }
}

bool
ParameterConflictHandler::conflict_exposure(const ParameterValueMap &p)
{
  // auto exposure must not be enabled while fixed exposure time is set
  return is_set(p, AE) && p.at(AE).get<bool>() && is_set(p, ET);
}
