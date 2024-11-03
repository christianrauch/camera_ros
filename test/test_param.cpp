#include "instantiate_component.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>


class Test : public testing::Test
{
protected:
  void
  SetUp() override
  {
    rclcpp::init(0, nullptr);

    exec = rclcpp::executors::SingleThreadedExecutor::make_shared();

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    camera = instantiate_component("camera::CameraNode", options);

    exec->add_node(camera.get_node_base_interface());
  }

  void
  TearDown() override
  {
    exec->remove_node(camera.get_node_base_interface());

    rclcpp::shutdown();
  }

private:
  rclcpp::Executor::SharedPtr exec;
  rclcpp_components::NodeInstanceWrapper camera;
};


TEST_F(Test, test)
{
  //
}

int
main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
