#include "ParameterHandler.hpp"
#include "clamp.hpp"
#include "cv_to_pv.hpp"
#include "parameter_conflict_check.hpp"
#include "type_extent.hpp"
#include <rclcpp/logging.hpp>


ParameterHandler::ParameterHandler(rclcpp::Node *const node)
    : node(node)
{
  //
}

void
ParameterHandler::declare(const libcamera::ControlInfoMap &controls)
{
  // dynamic camera configuration
  ParameterMap parameters_init;
  for (const auto &[id, info] : controls) {
    // store control id with name
    parameter_ids[id->name()] = id;

    if (info.min().numElements() != info.max().numElements())
      throw std::runtime_error("minimum and maximum parameter array sizes do not match");

    // check if the control can be mapped to a parameter
    const rclcpp::ParameterType pv_type = cv_to_pv_type(id);
    if (pv_type == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_WARN_STREAM(node->get_logger(), "unsupported control '" << id->name() << "'");
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
    rclcpp::ParameterValue value;
    try {
      value = cv_to_pv(clamp(info.def(), info.min(), info.max()));
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
                        "declare " << id->name() << " with default " << rclcpp::to_string(value));

    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      node->declare_parameter(id->name(), pv_type, param_descr);
    }
    else {
      node->declare_parameter(id->name(), value, param_descr);
      parameters_init[id->name()] = value;
    }
  }

  // ...

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
