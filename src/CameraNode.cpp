#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
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
};

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)

CameraNode::CameraNode(const rclcpp::NodeOptions &options) : Node("camera", options)
{
  camera_manager.start();

  for (const std::shared_ptr<libcamera::Camera> &camera : camera_manager.cameras()) {
    std::cout << camera->id() << std::endl;
  }
}

CameraNode::~CameraNode()
{
  camera_manager.stop();
}

} // namespace camera
