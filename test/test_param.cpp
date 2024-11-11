#include "instantiate_component.hpp"
#include "log_client.hpp"
#include "param_client.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>


class ParamTest : public testing::Test
{
protected:
  void
  SetUp() override
  {
    rclcpp::get_logger(CAMERA_NODE_NAME).set_level(rclcpp::Logger::Level::Debug);

    exec = rclcpp::executors::SingleThreadedExecutor::make_shared();

    log_client = std::make_unique<LogClient>(rclcpp::NodeOptions {}.use_intra_process_comms(true), CAMERA_NODE_NAME);

    param_client = std::make_unique<ParamClient>(rclcpp::NodeOptions {}.use_intra_process_comms(true), CAMERA_NODE_NAME, exec);

    exec->add_node(log_client->get_node_base_interface());
  }

  void
  TearDown() override
  {
    exec->remove_node(camera.get_node_base_interface());
    exec->remove_node(log_client->get_node_base_interface());
  }

  void
  spin_all()
  {
    // spin log nodes until all work is done
    exec->spin_all(std::chrono::nanoseconds {0});
  }

  void
  instantiate_camera(const std::vector<rclcpp::Parameter> &parameter_overrides)
  {
    camera = instantiate_component(
      "camera::CameraNode",
      rclcpp::NodeOptions {}
        .use_intra_process_comms(true)
        .arguments({"--ros-args", "--remap", "__node:=" + CAMERA_NODE_NAME})
        .parameter_overrides(parameter_overrides));

    exec->add_node(camera.get_node_base_interface());
  }

  const std::string CAMERA_NODE_NAME = "camera";

  rclcpp::Executor::SharedPtr exec;
  rclcpp_components::NodeInstanceWrapper camera;
  std::unique_ptr<ParamClient> param_client;
  std::unique_ptr<LogClient> log_client;
};


TEST_F(ParamTest, override_default)
{
  instantiate_camera({});

  spin_all();

  // expect: declare 'AeEnable' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search("declare AeEnable with default true"));
  ASSERT_TRUE(log_client->regex_search("declare ExposureTime with default (.*)"));

  // expect: only 'AeEnable' is set
  ASSERT_TRUE(log_client->regex_search("setting bool parameter AeEnable to true"));
  ASSERT_FALSE(log_client->regex_search("setting integer parameter ExposureTime to (.*)"));

  // check that camera has parameter 'AeEnable'
  ASSERT_TRUE(param_client->has_parameter("AeEnable"));
  // check that camera has parameter 'ExposureTime'
  ASSERT_TRUE(param_client->has_parameter("ExposureTime"));

  // parameters should still be set with their default values
  ASSERT_FALSE(param_client->get_parameters({"AeEnable", "ExposureTime"}).empty());
}


int
main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
