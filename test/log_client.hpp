#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>


class LogClient : public rclcpp::Node
{
public:
  LogClient(const rclcpp::NodeOptions &options,
            const std::string &camera_node_name)
      : Node("log_client", options),
        camera_node_name(camera_node_name)
  {
    sub_log = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::RosoutQoS {},
      std::bind(&LogClient::on_log, this, std::placeholders::_1));
  }

  void
  reset()
  {
    msgs.clear();
  }

  bool
  regex_search(const std::string &pattern)
  {
    std::regex re {pattern};
    for (const std::string &msg : msgs) {
      if (std::regex_search(msg, re))
        return true;
    }
    return false;
  }

private:
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_log;
  const std::string camera_node_name;
  std::list<std::string> msgs;

  void
  on_log(const rcl_interfaces::msg::Log::SharedPtr msg)
  {
    if (msg->name == camera_node_name)
      msgs.push_back(msg->msg);
  }
};
