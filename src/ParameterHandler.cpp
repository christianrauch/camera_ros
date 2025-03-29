#include "ParameterHandler.hpp"
#include "clamp.hpp"
#include "cv_to_pv.hpp"
#include "pv_to_cv.hpp"
#include "type_extent.hpp"
#include "types.hpp"
#include <cstddef>
#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>


#define CASE_RANGE(T, RU, RL)                                      \
  case libcamera::ControlType##T:                                  \
  {                                                                \
    rcl_interfaces::msg::RU range;                                 \
    range.from_value = max<libcamera::ControlType##T>(info.min()); \
    range.to_value = min<libcamera::ControlType##T>(info.max());   \
    descriptor.RL = {range};                                       \
  } break;

rcl_interfaces::msg::SetParametersResult
format_result(const std::vector<std::string> &msgs)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = msgs.empty();
  if (!result.successful) {
    for (size_t i = 0; i < msgs.size(); i++) {
      if (msgs.size() > 1)
        result.reason += "(" + std::to_string(i) + ") ";
      result.reason += msgs[i];
      if (i < msgs.size() - 1)
        result.reason += "; ";
    }
  }
  return result;
}

ParameterHandler::ParameterHandler(rclcpp::Node *const node)
    : node(node)
{
  param_cb_on = node->add_on_set_parameters_callback(
    std::bind(&ParameterHandler::OnSetValidate, this, std::placeholders::_1));
#ifdef RCLCPP_HAS_PARAM_EXT_CB
  param_cb_pre = node->add_pre_set_parameters_callback(
    std::bind(&ParameterHandler::PreSetResolve, this, std::placeholders::_1));
  param_cb_post = node->add_post_set_parameters_callback(
    std::bind(&ParameterHandler::PostSetApply, this, std::placeholders::_1));
#endif
}

void
ParameterHandler::declare(const libcamera::ControlInfoMap &controls)
{
  // convert camera controls to parameters:
  //  1. convert default control values to parameter values
  //  2. declare parameters without default values and without overrides
  //  3. resolve conflicts in default values
  //  4. resolve conflicts in overrides
  //  5. atomically set parameter values

  // All "control" parameters are declared as dynamically typed in order to be able
  // to unset them (set their type to 'rclcpp::ParameterType::PARAMETER_NOT_SET').
  // Unsetting a statically typed parameter causes "cannot undeclare a statically typed parameter".

  if (controls.empty()) {
    RCLCPP_DEBUG_STREAM(node->get_logger(), "No controls to declare.");
    return;
  }

  ParameterConflictHandler::ParameterValueMap parameters;
  for (const auto &[id, info] : controls) {
    if (info.min().numElements() != info.max().numElements()) {
      RCLCPP_WARN_STREAM(node->get_logger(),
                         id->name() << ": minimum (" << info.min().numElements() << ") and "
                                    << "maximum (" << info.max().numElements() << ") parameter "
                                    << "array sizes do not match");
      continue;
    }

    libcamera::ControlValue cv_def = info.def();

    // check if the control can be mapped to a parameter
    rclcpp::ParameterType pv_type;
    std::size_t extent;
    try {
      pv_type = cv_to_pv_type(id);
      extent = get_extent(id);
    }
    catch (const unknown_control &e) {
      // ignore not yet handled control
      RCLCPP_WARN_STREAM(node->get_logger(), e.what());
      continue;
    }
    catch (const unsupported_control &e) {
      // ignore control which cannot be supported
      RCLCPP_WARN_STREAM(node->get_logger(), e.what());
      continue;
    }

    const bool ctrl_scalar = (extent == 0);
    const bool ctrl_dynamic = (extent == libcamera::dynamic_extent);
    const bool ctrl_fixed = !(ctrl_scalar || ctrl_dynamic);

    if (ctrl_fixed && !cv_def.isArray() && !cv_def.isNone()) {
      RCLCPP_WARN_STREAM(
        node->get_logger(),
        id->name()
          << ": cannot set default scalar value '" << cv_def.toString() << "' "
          << "on span control (extend: " << extent << "), default will be ignored");
      cv_def = {};
    }

    // format type description
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = id->name();
    descriptor.type = pv_type;
    descriptor.dynamic_typing = true;
    const std::string cv_type_descr =
      ctrl_scalar ? "scalar" : "array[" + (ctrl_dynamic ? std::string() : std::to_string(extent)) + "]";
    descriptor.description =
      std::to_string(id->type()) + " " + cv_type_descr + " range {" + info.min().toString() +
      "}..{" + info.max().toString() + "}" +
      (cv_def.isNone() ? std::string {} : " (default: {" + cv_def.toString() + "})");

    // store descriptor for later re-declaration
    parameter_descriptors[id->name()] = descriptor;

    // Camera controls can have arrays for minimum and maximum values, but the parameter descriptor
    // only supports scalar bounds. Get smallest bounds for minimum and maximum set and warn user.
    if (info.min().isArray() || info.max().isArray()) {
      RCLCPP_WARN_STREAM(
        node->get_logger(),
        "unsupported control array bounds: {" << info.min().toString() << "} ... {" << info.max().toString() << "}");
    }

    switch (id->type()) {
      CASE_RANGE(Integer32, IntegerRange, integer_range)
      CASE_RANGE(Integer64, IntegerRange, integer_range)
      CASE_RANGE(Float, FloatingPointRange, floating_point_range)
#if LIBCAMERA_VER_GE(0, 4, 0)
      CASE_RANGE(Unsigned16, IntegerRange, integer_range)
      CASE_RANGE(Unsigned32, IntegerRange, integer_range)
#endif
    default:
      break;
    }

    // clamp default ControlValue to min/max range and cast to ParameterValue
    try {
      parameters[id->name()] = cv_to_pv(clamp(cv_def, info.min(), info.max()));
    }
    catch (const invalid_conversion &e) {
      RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "unsupported control '" << id->name() << "' (type: " << std::to_string(cv_def.type()) << "): " << e.what());
      continue;
    }

    // store supported controls with name, id and info
    camera_controls.emplace(id->name(), control_info_t {.id = id, .info = info});

    // declare parameters without default values, types and overrides to avoid triggering the callbacks
    RCLCPP_DEBUG_STREAM(node->get_logger(), "declare '" << id->name() << "' (type: " << pv_type << ")");
    node->declare_parameter(id->name(), rclcpp::ParameterValue {}, descriptor, true);
  }

  // resolve conflicts in default configuration
  const std::vector<std::string> msgs_defaults = parameter_conflict_handler.resolve_defaults(parameters);
  for (const std::string &msg : msgs_defaults) {
    RCLCPP_DEBUG_STREAM(node->get_logger(), "resolve defaults: " << msg);
  }

  const std::map<std::string, rclcpp::ParameterValue> &overrides =
    node->get_node_parameters_interface()->get_parameter_overrides();

  // apply parameter overrides and ignore those overrides,
  // which are not supported by the camera controls
  for (auto &[name, value] : parameters) {
    if (overrides.count(name))
      value = overrides.at(name);
  }

  // resolve conflicts in overridden configuration
  const std::vector<std::string> msgs_overrides = parameter_conflict_handler.resolve_overrides(parameters);
  for (const std::string &msg : msgs_overrides) {
    RCLCPP_WARN_STREAM(node->get_logger(), "resolve overrides: " << msg);
  }

  // convert parameters to list and set them atomically
  std::vector<rclcpp::Parameter> parameters_init_list;
  for (const auto &[name, value] : parameters) {
    parameters_init_list.emplace_back(name, value);
  }

  const rcl_interfaces::msg::SetParametersResult param_set_result =
    node->set_parameters_atomically(parameters_init_list);
  if (!param_set_result.successful)
    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot set parameters: " << param_set_result.reason);
}

const libcamera::ControlList &
ParameterHandler::get_control_values()
{
  const std::lock_guard<std::mutex> lock(control_values_lock);
  return control_values;
}

void
ParameterHandler::move_control_values(libcamera::ControlList &controls)
{
  // move the control values to the reference
  // this will clear the internal control values
  const std::lock_guard<std::mutex> lock(control_values_lock);
  controls = std::move(control_values);
}

void
ParameterHandler::redeclare()
{
  // rclcpp::Node::set_parameter() implicitly undeclares a parameter if its type
  // changes to rclcpp::PARAMETER_NOT_SET. Thus, we have to re-declare controls
  // as parameters.

  for (const auto &[name, _] : camera_controls) {
    if (!node->has_parameter(name)) {
      node->declare_parameter(name, rclcpp::ParameterValue {}, parameter_descriptors.at(name), true);
    }
  }
}

void
ParameterHandler::PreSetResolve(std::vector<rclcpp::Parameter> &parameters)
{
  parameter_conflict_handler.restore(parameters);
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::OnSetValidate(const std::vector<rclcpp::Parameter> &parameters)
{
#ifdef RCLCPP_HAS_PARAM_EXT_CB
  const rcl_interfaces::msg::SetParametersResult result = format_result(validate_new_parameters(parameters));
#else
  // If this is the only parameter callback available, call the pre-, on-, and post-set callbacks
  // manually on a writeable copy of the parameters.
  std::vector<rclcpp::Parameter> parameters2 = parameters;
  PreSetResolve(parameters2);
  const rcl_interfaces::msg::SetParametersResult result = format_result(validate_new_parameters(parameters2));
  if (result.successful)
    PostSetApply(parameters2);
#endif

  parameter_conflict_handler.store_commit_or_revert(result.successful);

  return result;
}

void
ParameterHandler::PostSetApply(const std::vector<rclcpp::Parameter> &parameters)
{
  control_values_lock.lock();

  // New parameter/control values are only applied when the next request callback
  // is called. When the parameter callback is called more than once in between
  // request callbacks, we will ignore the previous parameters and only ever apply
  // the most recent parameter values supplied before the next request callback.

  // If previous parameters have not been applied yet, clear the old parameters
  // and inform the user about it.
  if (!control_values.empty()) {
    std::string cv_list;
    for (const auto &[id, cv] : control_values) {
      cv_list += "  " + libcamera::controls::controls.at(id)->name() + ": " + cv.toString() + '\n';
    }
    RCLCPP_WARN_STREAM(node->get_logger(),
                       "previous parameters have note been apllied yet and will be ignored:"
                         << std::endl
                         << cv_list);
    control_values.clear();
  }

  for (const rclcpp::Parameter &parameter : parameters) {
    // filter parameters that are not camera controls
    if (!camera_controls.count(parameter.get_name()))
      continue;

    // filter unset values
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      continue;

    const libcamera::ControlId *id = camera_controls.at(parameter.get_name()).id;
    const libcamera::ControlValue value = pv_to_cv(parameter, id->type());
    control_values.set(id->id(), value);
  }

  control_values_lock.unlock();
}

std::vector<std::string>
ParameterHandler::validate_new_parameters(const std::vector<rclcpp::Parameter> &parameters)
{
  // store previous control values before new parameter updates are applied
  std::vector<std::string> names;
  for (const auto &[name, _] : camera_controls) {
    names.push_back(name);
  }
  const std::vector<rclcpp::Parameter> parameters_old = node->get_parameters(names);

  // check for conflicts caused by new parameters
  const std::vector<std::string> msgs = parameter_conflict_handler.check(parameters_old, parameters);
  if (!msgs.empty()) {
    return msgs;
  }

  // check new parameter value against control
  std::vector<std::string> msgs_valid_check;
  for (const rclcpp::Parameter &parameter : parameters) {
    // ignore non-controls
    if (!camera_controls.count(parameter.get_name()))
      continue;

    const libcamera::ControlId *id = camera_controls.at(parameter.get_name()).id;
    const rclcpp::ParameterType type_ctrl = cv_to_pv_type(id);
    const rclcpp::ParameterType type_param = parameter.get_type();

    // ignore unset parameters
    if (type_param == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      continue;
    }

    // check parameter type against the control type
    if (type_param != type_ctrl) {
      msgs_valid_check.push_back(
        parameter.get_name() + ": parameter types mismatch, expected '" +
        rclcpp::to_string(type_ctrl) + "', got '" + rclcpp::to_string(type_param) +
        "'");
      continue;
    }

    // check that the parameter value can be converted
    libcamera::ControlValue value;
    try {
      value = pv_to_cv(parameter, id->type());
    }
    catch (const invalid_conversion &e) {
      msgs_valid_check.push_back(e.what());
      continue;
    }

    // check converted type against control type
    assert(value.type() == id->type());

    // check array dimension
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

    // check bounds
    const libcamera::ControlInfo &ci = camera_controls.at(parameter.get_name()).info;
    if (value < ci.min() || value > ci.max()) {
      msgs_valid_check.push_back(
        "parameter value " + value.toString() + " outside of range: " + ci.toString());
      continue;
    }

    // success, show debug message
    RCLCPP_DEBUG_STREAM(node->get_logger(),
                        "setting " << parameter.get_type_name() << " parameter "
                                   << parameter.get_name() << " to "
                                   << parameter.value_to_string());
  }

  return msgs_valid_check;
}
