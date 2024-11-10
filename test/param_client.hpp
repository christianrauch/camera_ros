#include <rclcpp/node.hpp>
#include <rclcpp/parameter_client.hpp>


class ParamClient
{
public:
  ParamClient(const rclcpp::NodeOptions &options,
              const std::string &camera_node_name,
              rclcpp::Executor::SharedPtr executor)
      : node {rclcpp::Node::make_shared("param_client", options)},
        client {executor, node, camera_node_name}
  {
    //
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface()
  {
    return node->get_node_base_interface();
  }

  std::vector<std::string>
  list_parameters()
  {
    return client.list_parameters({}, {}).names;
  }

  bool
  has_parameter(const std::string &parameter_name)
  {
    return client.has_parameter(parameter_name);
  }

  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> &parameter_names)
  {
    return client.get_parameters(parameter_names);
  }

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::SyncParametersClient client;
};
