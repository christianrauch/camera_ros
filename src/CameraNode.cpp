#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/formats.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>


namespace camera
{
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions &options);

  ~CameraNode();

private:
  libcamera::CameraManager camera_manager;
  std::shared_ptr<libcamera::Camera> camera;
};

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)

CameraNode::CameraNode(const rclcpp::NodeOptions &options) : Node("camera", options)
{
  camera_manager.start();

  if (camera_manager.cameras().empty())
    throw std::runtime_error("no cameras available");

  // get the first camera
  camera = camera_manager.get(camera_manager.cameras().front()->id());
  if (!camera)
    throw std::runtime_error("failed to find first camera");

  if (camera->acquire())
    throw std::runtime_error("failed to acquire first camera");

  std::unique_ptr<libcamera::CameraConfiguration> cfg =
    camera->generateConfiguration({libcamera::StreamRole::StillCapture});
  if (!cfg)
    throw std::runtime_error("failed to generate still capture configuration");
  cfg->at(0).pixelFormat = libcamera::formats::RGB888;

  switch (cfg->validate()) {
  case libcamera::CameraConfiguration::Valid:
    break;
  case libcamera::CameraConfiguration::Adjusted:
    std::cerr << "Stream configuration adjusted" << std::endl;
    break;
  case libcamera::CameraConfiguration::Invalid:
    throw std::runtime_error("failed to valid stream configurations");
    break;
  }

  if (camera->configure(cfg.get()) < 0)
    throw std::runtime_error("failed to configure streams");

  if (camera->start())
    throw std::runtime_error("failed to start camera");
}

CameraNode::~CameraNode()
{
  if (camera->stop())
    std::cerr << "failed to stop camera" << std::endl;
  camera->release();
  camera_manager.stop();
}

} // namespace camera
