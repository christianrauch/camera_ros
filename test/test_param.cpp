#include "instantiate_component.hpp"
#include "param_client.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>


class ParamTest : public testing::Test
{
protected:
  void
  SetUp() override
  {
    rclcpp::init(0, nullptr);

    exec = rclcpp::executors::SingleThreadedExecutor::make_shared();

    // common node options
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    camera = instantiate_component("camera::CameraNode", options);
    client = std::make_unique<ParamClient>(options, camera.get_node_base_interface()->get_name(), exec);

    exec->add_node(camera.get_node_base_interface());
  }

  void
  TearDown() override
  {
    exec->remove_node(camera.get_node_base_interface());

    rclcpp::shutdown();
  }

  rclcpp::Executor::SharedPtr exec;
  rclcpp_components::NodeInstanceWrapper camera;
  std::unique_ptr<ParamClient> client;
};

TEST_F(ParamTest, any_param)
{
  // test that we have at least one node paramter
  const std::vector<std::string> parameter_list = client->list_parameters();
  ASSERT_FALSE(parameter_list.empty());
}

int
main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
