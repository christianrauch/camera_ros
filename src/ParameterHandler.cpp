#include "ParameterHandler.hpp"
#include "clamp.hpp"
#include "cv_to_pv.hpp"
// #include "parameter_conflict_check.hpp"
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


// init:
// 1. declare parameters without overrides and without default values
// 2. resolve conflicts between default values and overrides
// 3. apply

// runtime:
// 1. resolve conflicts, warn user
// 2. apply

typedef std::unordered_map<std::string, rclcpp::ParameterValue> ParamValueMap;

// typedef std::unordered_map<std::string, rclcpp::Parameter &> ParameterView;
typedef std::unordered_map<std::string, const rclcpp::Parameter &> ParameterViewConst;

// ParameterView
// param_view(std::vector<rclcpp::Parameter> &parameters)
// {
//   // create a mapping of parameter names to references of that parameter
//   ParameterView param_map;
//   for (rclcpp::Parameter &parameter : parameters) {
//     param_map.insert({parameter.get_name(), parameter});
//   }
//   return param_map;
// }

// std::vector<rclcpp::Parameter &>
// param_view(std::unordered_map<std::string, rclcpp::ParameterValue> &parameters)
// {
//   // create a mapping of parameter names to references of that parameter
//   // ParameterView param_map;
//   // for (rclcpp::Parameter &parameter : parameters) {
//   //   param_map.insert({parameter.get_name(), parameter});
//   // }
//   // return param_map;

//   for (auto &[name, value] : parameters) {
//     if (overrides.count(name))
//       value = overrides.at(name);
//   }
// }

ParameterViewConst
param_view(const std::vector<rclcpp::Parameter> &parameters)
{
  // create a mapping of parameter names to references of that parameter
  ParameterViewConst param_map;
  for (const rclcpp::Parameter &parameter : parameters) {
    param_map.insert({parameter.get_name(), parameter});
  }
  return param_map;
}

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

bool
conflict_exposure(const ParamValueMap &p)
{
  // auto exposure must not be enabled while fixed exposure time is set
  return p.count("AeEnable") && p.at("AeEnable").get<bool>() &&
         p.count("ExposureTime") && p.at("ExposureTime").get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
}

std::vector<std::string>
resolve_defaults(ParamValueMap &p)
{
  std::vector<std::string> msgs;

  // default: prefer auto exposure
  if (conflict_exposure(p)) {
    // disable exposure
    p.at("ExposureTime") = {};
    msgs.emplace_back("AeEnable and ExposureTime must not be enabled at the same time. 'ExposureTime' will be disabled.");
  }
  return msgs;
}

std::vector<std::string>
resolve_overrides(ParamValueMap &p)
{
  std::vector<std::string> msgs;

  // overrides: prefer provided exposure
  if (conflict_exposure(p)) {
    // disable auto exposure
    p.at("AeEnable") = rclcpp::ParameterValue {false};
    msgs.emplace_back("AeEnable and ExposureTime must not be enabled at the same time. 'AeEnable' will be set to off.");
  }
  return msgs;
}

std::vector<std::string>
check(const std::vector<rclcpp::Parameter> &parameters_old,
      const std::vector<rclcpp::Parameter> &parameters_new)
{
  std::vector<std::string> msgs;

  // initialise full parameter list with "new" parameters
  ParameterViewConst parameter_map = param_view(parameters_new);
  // move "old" parameters, which are not replaced by "new" parameters, to full parameter list
  parameter_map.merge(param_view(parameters_old));

  // ParameterMap parameter_map;

  // old configuration state
  // for (const auto &[name, value] : parameters_old)
  //   parameter_map[name] = value;

  // apply new configuration update
  // for (const auto &p : parameters_new)
  //   parameter_map[p.get_name()] = p.get_parameter_value();

  // is auto exposure going to be enabled?
  const bool ae_enabled =
    parameter_map.count("AeEnable") && parameter_map.at("AeEnable").as_bool();
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

ParameterHandler::ParameterHandler(rclcpp::Node *const node)
    : node(node)
{
  param_cb_on = node->add_on_set_parameters_callback(
    std::bind(&ParameterHandler::OnSetValidate, this, std::placeholders::_1));

#if defined(RCLCPP_HAS_PARAM_EXT_CB) && 0
  // pre_set -> on_set -> post_set

  // adjust parameters
  // std::function<void (std::vector<rclcpp::Parameter> &)>
  param_cb_pre = node->add_pre_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> &parameters) -> void {
      std::cout << "ParameterHandler pre >>" << std::endl;
      for (const rclcpp::Parameter &param : parameters) {
        std::cout << "ParameterHandler pre: " << param.get_name() << " to " << param.value_to_string() << std::endl;
      }
      std::cout << "ParameterHandler pre ##" << std::endl;
    });

  // validate parameters
  // std::function<rcl_interfaces::msg::SetParametersResult(const std::vector<rclcpp::Parameter> &)>
  param_cb_on = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
      std::cout << "ParameterHandler set >>" << std::endl;
      for (const rclcpp::Parameter &param : parameters) {
        std::cout << "ParameterHandler set: " << param.get_name() << " to " << param.value_to_string() << std::endl;
      }
      std::cout << "ParameterHandler set ##" << std::endl;
      rcl_interfaces::msg::SetParametersResult pr;
      pr.successful = true;
      return pr;
    });

  // apply parameters
  // std::function<void (const std::vector<rclcpp::Parameter> &)>
  param_cb_post = node->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &parameters) -> void {
      std::cout << "ParameterHandler post >>" << std::endl;
      for (const rclcpp::Parameter &param : parameters) {
        std::cout << "ParameterHandler post: " << param.get_name() << " to " << param.value_to_string() << std::endl;
      }
      std::cout << "ParameterHandler post ##" << std::endl;
    });
#endif

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
  // std::vector<rclcpp::Parameter> parameters;
  std::unordered_map<std::string, rclcpp::ParameterValue> parameters;
  // std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;

  // convert camera controls to parameters
  for (const auto &[id, info] : controls) {
    // store control id with name
    parameter_ids[id->name()] = id;
    parameter_info[id->name()] = info;

    std::cout << "# " << id->name() << ": " << info.toString() << ", " << info.def().toString() << std::endl;

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
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = id->name();
    descriptor.type = pv_type;
    try {
      const std::size_t extent = get_extent(id);
      const bool scalar = (extent == 0);
      const bool dynamic = (extent == libcamera::dynamic_extent);
      const std::string cv_type_descr =
        scalar ? "scalar" : "array[" + (dynamic ? std::string() : std::to_string(extent)) + "]";
      descriptor.description =
        std::to_string(id->type()) + " " + cv_type_descr + " range {" + info.min().toString() +
        "}..{" + info.max().toString() + "}" +
        (info.def().isNone() ? std::string {} : " (default: {" + info.def().toString() + "})");
    }
    catch (const std::runtime_error &e) {
      // ignore
      RCLCPP_WARN_STREAM(node->get_logger(), e.what());
      continue;
    }

    // Camera controls can have arrays for minimum and maximum values, but the parameter descriptor
    // only supports scalar bounds.
    // Get smallest bounds for minimum and maximum set but warn user.
    if (info.min().isArray() || info.max().isArray()) {
      RCLCPP_WARN_STREAM(
        node->get_logger(),
        "unsupported control array bounds: {" << info.min().toString() << "} ... {" << info.max().toString() << "}");
    }

    switch (id->type()) {
    case libcamera::ControlTypeInteger32:
    {
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = max<libcamera::ControlTypeInteger32>(info.min());
      range.to_value = min<libcamera::ControlTypeInteger32>(info.max());
      descriptor.integer_range = {range};
    } break;
    case libcamera::ControlTypeInteger64:
    {
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = max<libcamera::ControlTypeInteger64>(info.min());
      range.to_value = min<libcamera::ControlTypeInteger64>(info.max());
      descriptor.integer_range = {range};
    } break;
    case libcamera::ControlTypeFloat:
    {
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value = max<libcamera::ControlTypeFloat>(info.min());
      range.to_value = min<libcamera::ControlTypeFloat>(info.max());
      descriptor.floating_point_range = {range};
    } break;
    default:
      break;
    }

    // if (range_int.from_value != range_int.to_value)
    //   descriptor.integer_range = {range_int};
    // if (range_float.from_value != range_float.to_value)
    //   descriptor.floating_point_range = {range_float};

    // clamp default ControlValue to min/max range and cast ParameterValue
    try {
      // const rclcpp::ParameterValue value_default = cv_to_pv(clamp(info.def(), info.min(), info.max()));
      // parameters.emplace_back(id->name(), value_default);
      parameters[id->name()] = cv_to_pv(clamp(info.def(), info.min(), info.max()));
      // descriptors.push_back(descriptor);
    }
    catch (const invalid_conversion &e) {
      RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "unsupported control '" << id->name() << "' (type: " << std::to_string(info.def().type()) << "): " << e.what());
      continue;
    }

    // declare parameters without default values and overrides to avoid triggering the callbacks already
    RCLCPP_DEBUG_STREAM(node->get_logger(), "declare '" << id->name() << "' (type: " << pv_type << ")");
    node->declare_parameter(id->name(), pv_type, descriptor, true);
  }

  // TODO: merge default parameters and overrides and resolve conflicts

  // std::vector<rclcpp::Parameter> parameters;
  std::cout << "defaults:" << std::endl;
  for (const auto &[name, value] : parameters) {
    std::cout << " " << name << ": " << rclcpp::to_string(value) << std::endl;
  }

  // resolve conflicts in default configuration
  const std::vector<std::string> msgs_defaults = resolve_defaults(parameters);
  for (const std::string &msg : msgs_defaults) {
    RCLCPP_DEBUG_STREAM(node->get_logger(), "resolve defaults: " << msg);
  }

  std::cout << "defaults (resolved):" << std::endl;
  for (const auto &[name, value] : parameters) {
    std::cout << " " << name << ": " << rclcpp::to_string(value) << std::endl;
  }

  const std::map<std::string, rclcpp::ParameterValue> &overrides =
    node->get_node_parameters_interface()->get_parameter_overrides();

  std::cout << "overrides:" << std::endl;
  for (const auto &[name, value] : overrides) {
    std::cout << " " << name << ": " << rclcpp::to_string(value) << std::endl;
  }

  // apply parameter overrides and ignore those,
  // which are not supported by the camera controls
  for (auto &[name, value] : parameters) {
    if (overrides.count(name))
      value = overrides.at(name);
  }

  // resolve conflicts in overridden configuration
  const std::vector<std::string> msgs_overrides = resolve_overrides(parameters);
  for (const std::string &msg : msgs_overrides) {
    RCLCPP_WARN_STREAM(node->get_logger(), "resolve overrides: " << msg);
  }

  // TODO: setting auto-exposure to off here, should restore the default exposure time

  std::cout << "defaults (override):" << std::endl;
  for (const auto &[name, value] : parameters) {
    std::cout << " " << name << ": " << rclcpp::to_string(value) << std::endl;
  }

  // convert parameters to list and set them atomically
  std::vector<rclcpp::Parameter> parameters_init_list;
  for (const auto &[name, value] : parameters) {
    if (value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
      parameters_init_list.emplace_back(name, value);
  }

  const rcl_interfaces::msg::SetParametersResult param_set_result =
    node->set_parameters_atomically(parameters_init_list);
  if (!param_set_result.successful)
    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot set parameters: " << param_set_result.reason);


  // std::vector<rclcpp::Parameter> aa = node->get_parameters({"AeEnable", "ExposureTime"});

  // resolve conflicts in default control values
  // TODO: let conflicts be resolved with callback?
  // NO? override ExposureTime must set (override) AeEnable false
  // ??? can we replace 'parameters_full' by runtime check of current param values?

  // resolve conflicts of default libcamera configuration and user provided overrides
  // std::vector<std::string> status;
  // std::tie(parameters_init, status) = resolve_conflicts(parameters, overrides);

  // for (const std::string &s : status)
  //   RCLCPP_WARN_STREAM(node->get_logger(), s);

  // std::vector<rclcpp::Parameter> parameters_init_list;
  // for (const auto &[name, value] : parameters_init)
  //   parameters_init_list.emplace_back(name, value);
  // node->set_parameters(parameters_init_list);
}

libcamera::ControlList &
ParameterHandler::get()
{
  const std::lock_guard<std::mutex> lock(parameters_lock);
  // TODO: final check of conflicts for gathered controls?
  return control_values;
}

void
ParameterHandler::clear()
{
  parameters_lock.lock();
  control_values.clear();
  parameters_lock.unlock();
}

void
ParameterHandler::adjust(std::vector<rclcpp::Parameter> &parameters)
{
  //
  (void)parameters;
  // for (const rclcpp::Parameter &parameter : parameters) {
  //   //
  // }
}

std::vector<std::string>
ParameterHandler::validate(const std::vector<rclcpp::Parameter> &parameters)
{
  // TODO: should just go over "controls"
  // const std::vector<std::string> parameter_names_old = node->list_parameters({}, {}).names;
  // std::vector<std::string> parameter_names_old;
  // for (const auto &[name, id] : parameter_ids) {
  //   parameter_names_old.push_back(name);
  // }

  // const std::vector<rclcpp::ParameterType> parameter_types_old = node->get_parameter_types(parameter_names_old);
  // rclcpp::ParameterType::PARAMETER_NOT_SET;
  // terminate called after throwing an instance of 'rclcpp::exceptions::ParameterUninitializedException'
  //   what():  parameter 'AeEnable' is not initialized
  std::vector<rclcpp::Parameter> parameters_old;
  // for (const auto &[name, id] : parameter_ids) {
  //   parameters_old.push_back(node->get_parameter(name));
  // }
  for (const auto &[name, id] : parameter_ids) {
    // auto desr = node->describe_parameter(name);
    // if (node->describe_parameter(name).type == rclcpp::ParameterType::PARAMETER_NOT_SET)
    //   continue;
    // parameters_old.push_back(node->get_parameter(name));
    rclcpp::Parameter p;
    node->get_parameter(name, p);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      continue;
    parameters_old.push_back(p);
  }
  // for (const auto &[name, id] : parameter_ids) {
  //   node->get_parameter(name, );
  // }
  // const std::vector<rclcpp::Parameter> parameters_old = node->get_parameters(parameter_names_old);

  // conflicts
  const std::vector<std::string> msgs = check(parameters_old, parameters);
  // check(parameters_old, parameters)
  // const rcl_interfaces::msg::SetParametersResult result = format_result();
  if (!msgs.empty()) {
    // std::cout << "validate: " << result.reason << std::endl;
    // return format_result(msgs);
    return msgs;
  }
  // if (!result.successful)
  //   return result;

  // TODO: check other mismatches from 'parameterCheckAndConvert'

  std::vector<std::string> msgs_valid_check;
  for (const rclcpp::Parameter &parameter : parameters) {
    // ignore non-controls
    if (!parameter_ids.count(parameter.get_name()))
      continue;

    RCLCPP_DEBUG_STREAM(node->get_logger(),
                        "setting " << parameter.get_type_name() << " parameter "
                                   << parameter.get_name() << " to "
                                   << parameter.value_to_string());

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

      // TODO: value clamping into pre_set

      // TODO: set control_values in post_set callback
      // control_values[parameter_ids.at(parameter.get_name())->id()] = value;
      // parameters_full[parameter.get_name()] = parameter.get_parameter_value();
    } // not None
  }

  // return format_result(msgs_valid_check);

  return msgs_valid_check;
}

void
ParameterHandler::apply(const std::vector<rclcpp::Parameter> &parameters)
{
  std::cout << "apply..." << std::endl;

  // NOTE: apply could be called multiple times before 'control_values' is read,
  // should we clear 'control_values' on every apply to keep previous checks valid?

  // TODO: use a callback to set controls immediately on request

  parameters_lock.lock();
  // control_values.clear(); // need this??
  for (const rclcpp::Parameter &parameter : parameters) {
    if (!parameter_ids.count(parameter.get_name()))
      continue;
    const libcamera::ControlId *id = parameter_ids.at(parameter.get_name());
    const libcamera::ControlValue value = pv_to_cv(parameter, id->type());
    // control_values[parameter_ids.at(parameter.get_name())->id()] = value;
    // const std::string &name = libcamera::controls::controls.at(id)->name();
    RCLCPP_DEBUG_STREAM(node->get_logger(), "apply " << id->name() << ": " << value.toString());
    control_values.set(parameter_ids[parameter.get_name()]->id(), value);
    // TODO: What if 'control_values' gets conflcit here? Should we gather this before?
  }
  parameters_lock.unlock();
}

rcl_interfaces::msg::SetParametersResult
ParameterHandler::OnSetValidate(const std::vector<rclcpp::Parameter> &parameters)
{
#ifdef RCLCPP_HAS_PARAM_EXT_CB
  const rcl_interfaces::msg::SetParametersResult result = format_result(validate(parameters));
#else
  // If this is the only parameter callback available, call the pre-, on- and post-set callbacks
  // manually on a writable copy of the paramters.
  std::vector<rclcpp::Parameter> parameters2 = parameters;
  adjust(parameters2);
  const rcl_interfaces::msg::SetParametersResult result = format_result(validate(parameters2));
  if (result.successful)
    apply(parameters2);
#endif
  return result;
}

#ifdef RCLCPP_HAS_PARAM_EXT_CB
void
ParameterHandler::PreSetResolve(std::vector<rclcpp::Parameter> &parameters)
{
  adjust(parameters);
}

void
ParameterHandler::PostSetApply(const std::vector<rclcpp::Parameter> &parameters)
{
  apply(parameters);
}
#endif

// std::tuple<ParameterHandler::ControlValueMap, std::vector<std::string>>
// ParameterHandler::parameterCheckAndConvert(const std::vector<rclcpp::Parameter> &parameters)
// {
//   // check target parameter state (current and new parameters)
//   // for conflicting configuration
//   // const std::vector<std::string> msgs_conflicts = check_conflicts(parameters, parameters_full);
//   // if (!msgs_conflicts.empty()) {
//   //   return {ControlValueMap {}, msgs_conflicts};
//   // }

//   ControlValueMap control_values;
//   std::vector<std::string> msgs_valid_check;

//   // for (const rclcpp::Parameter &parameter : parameters) {
//   //   RCLCPP_DEBUG_STREAM(node->get_logger(),
//   //                       "setting " << parameter.get_type_name() << " parameter "
//   //                                  << parameter.get_name() << " to "
//   //                                  << parameter.value_to_string());

//   //   if (parameter_ids.count(parameter.get_name())) {
//   //     const libcamera::ControlId *id = parameter_ids.at(parameter.get_name());
//   //     const libcamera::ControlValue value = pv_to_cv(parameter, id->type());

//   //     if (!value.isNone()) {
//   //       // verify parameter type and dimension against default
//   //       const libcamera::ControlInfo &ci = parameter_info.at(parameter.get_name());

//   //       if (value.type() != id->type()) {
//   //         msgs_valid_check.push_back(
//   //           parameter.get_name() + ": parameter types mismatch, expected '" +
//   //           std::to_string(id->type()) + "', got '" + std::to_string(value.type()) +
//   //           "'");
//   //         continue;
//   //       }

//   //       const std::size_t extent = get_extent(id);
//   //       if (value.isArray() &&
//   //           (extent != libcamera::dynamic_extent) &&
//   //           (value.numElements() != extent))
//   //       {
//   //         msgs_valid_check.push_back(
//   //           parameter.get_name() + ": array dimensions mismatch, expected " +
//   //           std::to_string(extent) + ", got " + std::to_string(value.numElements()));
//   //         continue;
//   //       }

//   //       // check bounds and return error
//   //       if (value < ci.min() || value > ci.max()) {
//   //         msgs_valid_check.push_back(
//   //           "parameter value " + value.toString() + " outside of range: " + ci.toString());
//   //         continue;
//   //       }

//   //       control_values[parameter_ids.at(parameter.get_name())->id()] = value;
//   //       parameters_full[parameter.get_name()] = parameter.get_parameter_value();
//   //     }  // not None
//   //   }  // in parameter_ids
//   // }  // parameters

//   return {control_values, msgs_valid_check};
// }
