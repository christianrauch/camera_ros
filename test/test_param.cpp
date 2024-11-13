#include "instantiate_component.hpp"
#include "log_client.hpp"
#include "param_client.hpp"
#include <gtest/gtest.h>
#include <libcamera/camera_manager.h>
#include <rclcpp/rclcpp.hpp>


class ParamTest : public testing::Test
{
protected:
  void
  SetUp() override
  {
    {
      // skip tests when no camera is available
      libcamera::CameraManager camera_manager;
      camera_manager.start();
      if (camera_manager.cameras().empty())
        GTEST_SKIP() << "No cameras available. Skipping tests." << std::endl;
    }

    rclcpp::get_logger(CAMERA_NODE_NAME).set_level(rclcpp::Logger::Level::Debug);

    exec = rclcpp::executors::SingleThreadedExecutor::make_shared();

    log_client = std::make_unique<LogClient>(rclcpp::NodeOptions {}.use_intra_process_comms(true), CAMERA_NODE_NAME);

    param_client = std::make_unique<ParamClient>(rclcpp::NodeOptions {}.use_intra_process_comms(true), CAMERA_NODE_NAME, exec);

    exec->add_node(log_client->get_node_base_interface());
  }

  void
  TearDown() override
  {
    if (exec) {
      exec->remove_node(camera.get_node_base_interface());
      exec->remove_node(log_client->get_node_base_interface());
    }
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
    if (!exec) {
      FAIL() << "Executor is not initialised";
    }

    camera = instantiate_component(
      "camera::CameraNode",
      rclcpp::NodeOptions {}
        .use_intra_process_comms(true)
        .arguments({"--ros-args", "--remap", "__node:=" + CAMERA_NODE_NAME})
        .parameter_overrides(parameter_overrides));

    exec->add_node(camera.get_node_base_interface());
  }

  const std::string CAMERA_NODE_NAME = "camera";

  const std::string conflict_reason = "AeEnable and ExposureTime must not be set simultaneously";

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

TEST_F(ParamTest, override_ae_disabled)
{
  instantiate_camera({{"AeEnable", false}});

  spin_all();

  // expect: declare 'AeEnable' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search("declare AeEnable with default true"));
  ASSERT_TRUE(log_client->regex_search("declare ExposureTime with default (.*)"));

  // expect: only 'AeEnable' is set to override value
  ASSERT_TRUE(log_client->regex_search("setting bool parameter AeEnable to false"));
  ASSERT_FALSE(log_client->regex_search("setting integer parameter ExposureTime to (.*)"));

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), false);
}

TEST_F(ParamTest, override_exposure)
{
  instantiate_camera({{"ExposureTime", 15600}});

  spin_all();

  // expect: declare 'AeEnable' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search("declare AeEnable with default true"));
  ASSERT_TRUE(log_client->regex_search("declare ExposureTime with default (.*)"));

  // expect: 'AeEnable' adjusted to 'false'
  // expect: 'ExposureTime' set to override value
  ASSERT_TRUE(log_client->regex_search("setting bool parameter AeEnable to false"));
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to 15600"));

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), false);
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), 15600);
}

TEST_F(ParamTest, override_ae_disabled_exposure)
{
  instantiate_camera({{"AeEnable", false}, {"ExposureTime", 15600}});

  spin_all();

  // expect: declare 'AeEnable' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search("declare AeEnable with default true"));
  ASSERT_TRUE(log_client->regex_search("declare ExposureTime with default (.*)"));

  // expect: 'AeEnable' and 'ExposureTime' are set
  ASSERT_TRUE(log_client->regex_search("setting bool parameter AeEnable to false"));
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to 15600"));

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), false);
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), 15600);
}

TEST_F(ParamTest, override_ae_enabled_exposure)
{
  instantiate_camera({{"AeEnable", true}, {"ExposureTime", 15600}});

  spin_all();

  // expect: declare 'AeEnable' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search("declare AeEnable with default true"));
  ASSERT_TRUE(log_client->regex_search("declare ExposureTime with default (.*)"));

  // expect: 'ExposureTime' override with default 'AeEnable' shows warning
  ASSERT_TRUE(log_client->regex_search("AeEnable and ExposureTime must not be enabled at the same time. 'AeEnable' will be set to off."));

  // expect: 'AeEnable' and 'ExposureTime' are set
  // expect: 'AeEnable' adjusted to 'false'
  ASSERT_TRUE(log_client->regex_search("setting bool parameter AeEnable to false"));
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to 15600"));

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), false);
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), 15600);
}

TEST_F(ParamTest, override_default_set_exposure)
{
  instantiate_camera({});

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  // by default, auto exposure is active
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), true);

  const int exp_init = param_client->get_parameters({"ExposureTime"}).front().as_int();

  // setting 'ExposureTime' with 'AeEnable' enabled by default causes conflict
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_exposure =
    param_client->set_parameters({{"ExposureTime", exp_tar1}});
  ASSERT_FALSE(res_exposure[0].successful);
  ASSERT_EQ(res_exposure[0].reason, "AeEnable and ExposureTime must not be set simultaneously");
  // parameter updates are not applied
  ASSERT_NE(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_tar1);
}

TEST_F(ParamTest, override_ae_disabled_set_exposure)
{
  // disable 'AeEnable'
  instantiate_camera({{"AeEnable", false}});

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  // 'AeEnable' takes the override value
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), false);

  const int exp_init = param_client->get_parameters({"ExposureTime"}).front().as_int();

  // setting 'ExposureTime' does not cause conflict and is applied
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_exposure =
    param_client->set_parameters({{"ExposureTime", exp_tar1}});
  ASSERT_TRUE(res_exposure[0].successful);
  ASSERT_EQ(res_exposure[0].reason, std::string {});
  // parameter updates are applied
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_tar1);
}

TEST_F(ParamTest, override_ae_disabled_set_atom_ae_enabled_exposure)
{
  // disable 'AeEnable'
  instantiate_camera({{"AeEnable", false}});

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  const int exp_init = param_client->get_parameters({"ExposureTime"}).front().as_int();

  // setting 'AeEnable' and 'ExposureTime' at once will fail
  // no parameter updates are applied
  const std::vector<rclcpp::Parameter> initial_param_vals =
    param_client->get_parameters({"AeEnable", "ExposureTime"});
  const int exp_tar1 = exp_init + 100;
  const rcl_interfaces::msg::SetParametersResult res_atom =
    param_client->set_parameters_atomically({{"AeEnable", true}, {"ExposureTime", exp_tar1}});
  ASSERT_FALSE(res_atom.successful);
  ASSERT_EQ(res_atom.reason, conflict_reason);
  // parameters do not change
  ASSERT_EQ(initial_param_vals, param_client->get_parameters({"AeEnable", "ExposureTime"}));
}

TEST_F(ParamTest, override_ae_disabled_set_atom_exposure_ae_enabled)
{
  // disable 'AeEnable'
  instantiate_camera({{"AeEnable", false}});

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  const int exp_init = param_client->get_parameters({"ExposureTime"}).front().as_int();

  // setting 'ExposureTime' and 'AeEnable' at once will fail
  // no parameter updates are applied
  const std::vector<rclcpp::Parameter> initial_param_vals =
    param_client->get_parameters({"AeEnable", "ExposureTime"});
  const int exp_tar1 = exp_init + 100;
  const rcl_interfaces::msg::SetParametersResult res_atom =
    param_client->set_parameters_atomically({{"ExposureTime", exp_tar1}, {"AeEnable", true}});
  ASSERT_FALSE(res_atom.successful);
  ASSERT_EQ(res_atom.reason, conflict_reason);
  // parameters do not change
  ASSERT_EQ(initial_param_vals, param_client->get_parameters({"AeEnable", "ExposureTime"}));
}

TEST_F(ParamTest, override_ae_disabled_set_indiv_ae_enabled_exposure)
{
  // disable 'AeEnable'
  instantiate_camera({{"AeEnable", false}});

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  const int exp_init = param_client->get_parameters({"ExposureTime"}).front().as_int();

  // setting 'AeEnable' and 'ExposureTime' individually one-by-one will fail evenetually
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_indiv =
    param_client->set_parameters({{"AeEnable", true}, {"ExposureTime", exp_tar1}});
  // first parameter 'AeEnable' does not cause conflicts
  ASSERT_TRUE(res_indiv[0].successful);
  ASSERT_EQ(res_indiv[0].reason, std::string {});
  // second parameter 'ExposureTime' causes conflict with previous 'AeEnable'
  ASSERT_FALSE(res_indiv[1].successful);
  ASSERT_EQ(res_indiv[1].reason, conflict_reason);
  // only the parameter update for 'AeEnable' will have been applied
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), true);
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_init);
  ASSERT_NE(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_tar1);

  // setting 'ExposureTime' again will fail since 'AeEnable' has already been applied
  const int exp_tar2 = exp_init + 200;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_exposure =
    param_client->set_parameters({{"ExposureTime", exp_tar2}});
  ASSERT_FALSE(res_exposure[0].successful);
  ASSERT_EQ(res_exposure[0].reason, conflict_reason);
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_init);
  ASSERT_NE(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_tar2);
}

TEST_F(ParamTest, override_ae_disabled_set_indiv_exposure_ae_enabled)
{
  // disable 'AeEnable'
  instantiate_camera({{"AeEnable", false}});

  ASSERT_TRUE(param_client->is_set_parameter("AeEnable"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  const int exp_init = param_client->get_parameters({"ExposureTime"}).front().as_int();

  // setting 'ExposureTime' and 'AeEnable' individually one-by-one will fail evenetually
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_indiv =
    param_client->set_parameters({{"ExposureTime", exp_tar1}, {"AeEnable", true}});
  // first parameter 'ExposureTime' does not cause conflicts
  ASSERT_TRUE(res_indiv[0].successful);
  ASSERT_EQ(res_indiv[0].reason, std::string {});
  // second parameter 'AeEnable' takes precedence over previous 'ExposureTime'
  ASSERT_TRUE(res_indiv[1].successful);
  ASSERT_EQ(res_indiv[1].reason, std::string {});
  // both parameters will have been updated, but only 'AeEnable' will be effective
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_tar1);
  ASSERT_EQ(param_client->get_parameters({"AeEnable"}).front().as_bool(), true);
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
