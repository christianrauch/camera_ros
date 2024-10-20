#include "ParameterHandler.hpp"
#include "clamp.hpp"
#include "cv_to_pv.hpp"
#include "parameter_conflict_check.hpp"
#include "pv_to_cv.hpp"
#include "type_extent.hpp"
#include "types.hpp"
#include <cstddef>
#include <libcamera/base/span.h>
#include <libcamera/controls.h>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <stdexcept>


ParameterHandler::ParameterHandler(rclcpp::Node *const node)
    : node(node)
{
  //
}

void
ParameterHandler::declareFromControls(const libcamera::ControlInfoMap &controls)
{
  // dynamic camera configuration
  ParameterMap parameters_init;
  for (const auto &[id, info] : controls) {
    // store control id with name
    parameter_ids[id->name()] = id;
    parameter_info[id->name()] = info;

    if (info.min().numElements() != info.max().numElements())
      throw std::runtime_error("minimum and maximum parameter array sizes do not match");

    // check if the control can be mapped to a parameter
    rclcpp::ParameterType pv_type;
    try {
      pv_type = cv_to_pv_type(id);
      if (pv_type == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        RCLCPP_WARN_STREAM(node->get_logger(), "unsupported control '" << id->name() << "'");
        continue;
      }
    }
    catch (const std::runtime_error &e) {
      // ignore
      RCLCPP_WARN_STREAM(node->get_logger(), e.what());
      continue;
    }

    // format type description
    rcl_interfaces::msg::ParameterDescriptor param_descr;
    try {
      const std::size_t extent = get_extent(id);
      const bool scalar = (extent == 0);
      const bool dynamic = (extent == libcamera::dynamic_extent);
      const std::string cv_type_descr =
        scalar ? "scalar" : "array[" + (dynamic ? std::string() : std::to_string(extent)) + "]";
      param_descr.description =
        std::to_string(id->type()) + " " + cv_type_descr + " range {" + info.min().toString() +
        "}..{" + info.max().toString() + "}" +
        (info.def().isNone() ? std::string {} : " (default: {" + info.def().toString() + "})");
    }
    catch (const std::runtime_error &e) {
      // ignore
      RCLCPP_WARN_STREAM(node->get_logger(), e.what());
      continue;
    }

    // get smallest bounds for minimum and maximum set
    rcl_interfaces::msg::IntegerRange range_int;
    rcl_interfaces::msg::FloatingPointRange range_float;

    switch (id->type()) {
    case libcamera::ControlTypeInteger32:
      range_int.from_value = max<libcamera::ControlTypeInteger32>(info.min());
      range_int.to_value = min<libcamera::ControlTypeInteger32>(info.max());
      break;
    case libcamera::ControlTypeInteger64:
      range_int.from_value = max<libcamera::ControlTypeInteger64>(info.min());
      range_int.to_value = min<libcamera::ControlTypeInteger64>(info.max());
      break;
    case libcamera::ControlTypeFloat:
      range_float.from_value = max<libcamera::ControlTypeFloat>(info.min());
      range_float.to_value = min<libcamera::ControlTypeFloat>(info.max());
      break;
    default:
      break;
    }

    if (range_int.from_value != range_int.to_value)
      param_descr.integer_range = {range_int};
    if (range_float.from_value != range_float.to_value)
      param_descr.floating_point_range = {range_float};

    // clamp default ControlValue to min/max range and cast ParameterValue
    rclcpp::ParameterValue value_default;
    try {
      value_default = cv_to_pv(clamp(info.def(), info.min(), info.max()));
    }
    catch (const invalid_conversion &e) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "unsupported control '"
                            << id->name()
                            << "' (type: " << std::to_string(info.def().type()) << "): "
                            << e.what());
      continue;
    }

    // declare parameters and set default or initial value
    RCLCPP_DEBUG_STREAM(node->get_logger(),
                        "declare " << id->name() << " with default " << rclcpp::to_string(value_default));

    if (value_default.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      node->declare_parameter(id->name(), pv_type, param_descr);
    }
    else {
      node->declare_parameter(id->name(), value_default, param_descr);
      parameters_init[id->name()] = value_default;
    }
  }

  // resolve conflicts of default libcamera configuration and user provided overrides
  std::vector<std::string> status;
  std::tie(parameters_init, status) =
    resolve_conflicts(parameters_init, node->get_node_parameters_interface()->get_parameter_overrides());

  for (const std::string &s : status)
    RCLCPP_WARN_STREAM(node->get_logger(), s);

  std::vector<rclcpp::Parameter> parameters_init_list;
  for (const auto &[name, value] : parameters_init)
    parameters_init_list.emplace_back(name, value);
  node->set_parameters(parameters_init_list);
}

std::tuple<ParameterHandler::ControlValueMap, std::vector<std::string>>
ParameterHandler::parameterCheckAndConvert(const std::vector<rclcpp::Parameter> &parameters)
{
  // check target parameter state (current and new parameters)
  // for conflicting configuration
  const std::vector<std::string> msgs_conflicts = check_conflicts(parameters, parameters_full);
  if (!msgs_conflicts.empty()) {
    return {ControlValueMap {}, msgs_conflicts};
  }

  ControlValueMap control_values;
  std::vector<std::string> msgs_valid_check;

  for (const rclcpp::Parameter &parameter : parameters) {
    RCLCPP_DEBUG_STREAM(node->get_logger(),
                        "setting " << parameter.get_type_name() << " parameter "
                                   << parameter.get_name() << " to "
                                   << parameter.value_to_string());

    if (parameter_ids.count(parameter.get_name())) {
      const libcamera::ControlId *id = parameter_ids.at(parameter.get_name());
      const libcamera::ControlValue value = pv_to_cv(parameter, id->type());

      if (!value.isNone()) {
        // verify parameter type and dimension against default
        const libcamera::ControlInfo &ci = parameter_info.at(parameter.get_name());

        if (value.type() != id->type()) {
          msgs_valid_check.push_back(
            parameter.get_name() + ": parameter types mismatch, expected '" +
            std::to_string(id->type()) + "', got '" + std::to_string(value.type()) +
            "'");
          continue;
        }

        const std::size_t extent = get_extent(id);
        if (value.isArray() &&
            (extent != libcamera::dynamic_extent) &&
            (value.numElements() != extent))
        {
          msgs_valid_check.push_back(
            parameter.get_name() + ": array dimensions mismatch, expected " +
            std::to_string(extent) + ", got " + std::to_string(value.numElements()));
          continue;
        }

        // check bounds and return error
        if (value < ci.min() || value > ci.max()) {
          msgs_valid_check.push_back(
            "parameter value " + value.toString() + " outside of range: " + ci.toString());
          continue;
        }

        control_values[parameter_ids.at(parameter.get_name())->id()] = value;
        parameters_full[parameter.get_name()] = parameter.get_parameter_value();
      }
    }
  }

  return {control_values, msgs_valid_check};
}
