#include <cstring>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sys/mman.h>


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
  std::shared_ptr<libcamera::FrameBufferAllocator> allocator;
  std::vector<std::unique_ptr<libcamera::Request>> requests;

  // timestamp offset (ns) from camera time to system time
  int64_t time_offset = 0;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image_compressed;

  void requestComplete(libcamera::Request *request);
};

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)

CameraNode::CameraNode(const rclcpp::NodeOptions &options) : Node("camera", options)
{
  // pixel format
  rcl_interfaces::msg::ParameterDescriptor param_descr_format;
  param_descr_format.description = "pixel format of streaming buffer";
  param_descr_format.read_only = true;
  declare_parameter<std::string>("format", {}, param_descr_format);

  // publisher for raw and compressed image
  pub_image = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", 1);
  pub_image_compressed =
    this->create_publisher<sensor_msgs::msg::CompressedImage>("~/image_raw/compressed", 1);

  // start camera manager and check for cameras
  camera_manager.start();
  if (camera_manager.cameras().empty())
    throw std::runtime_error("no cameras available");

  // get the first camera
  camera = camera_manager.get(camera_manager.cameras().front()->id());
  if (!camera)
    throw std::runtime_error("failed to find first camera");

  if (camera->acquire())
    throw std::runtime_error("failed to acquire first camera");

  // configure camera stream
  std::unique_ptr<libcamera::CameraConfiguration> cfg =
    camera->generateConfiguration({libcamera::StreamRole::VideoRecording});

  if (!cfg)
    throw std::runtime_error("failed to generate configuration");

  // show all supported stream configurations and pixel formats
  std::cout << ">> stream configurations:" << std::endl;
  for (size_t i = 0; i < cfg->size(); i++) {
    const libcamera::StreamConfiguration &scfg = cfg->at(i);
    const libcamera::StreamFormats &formats = scfg.formats();

    std::cout << i << ": " << scfg.toString() << std::endl;
    for (const libcamera::PixelFormat &pixelformat : formats.pixelformats()) {
      std::cout << "  - Pixelformat: " << pixelformat.toString() << " ("
                << formats.range(pixelformat).min.toString() << " - "
                << formats.range(pixelformat).max.toString() << ")" << std::endl;
      std::cout << "    Sizes:" << std::endl;
      for (const libcamera::Size &size : formats.sizes(pixelformat))
        std::cout << "     - " << size.toString() << std::endl;
    }
  }

  libcamera::StreamConfiguration &scfg = cfg->at(0);
  const std::string format = get_parameter("format").as_string();
  // get pixel format from provided string or use first available format otherwise
  if (format.empty())
    scfg.pixelFormat = scfg.formats().pixelformats().at(0);
  else
    scfg.pixelFormat = libcamera::PixelFormat::fromString(format);

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

  std::cout << "camera \"" << camera->id() << "\" configured with " << scfg.toString() << " stream"
            << std::endl;

  // allocate stream buffers and create one request per buffer
  libcamera::Stream *stream = scfg.stream();

  allocator = std::make_shared<libcamera::FrameBufferAllocator>(camera);
  allocator->allocate(stream);

  for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator->buffers(stream)) {
    std::unique_ptr<libcamera::Request> request = camera->createRequest();
    if (!request)
      throw std::runtime_error("Can't create request");

    if (request->addBuffer(stream, buffer.get()) < 0)
      throw std::runtime_error("Can't set buffer for request");

    requests.push_back(std::move(request));
  }

  // register callback
  camera->requestCompleted.connect(this, &CameraNode::requestComplete);

  // start camera and queue all requests
  if (camera->start())
    throw std::runtime_error("failed to start camera");

  for (std::unique_ptr<libcamera::Request> &request : requests)
    camera->queueRequest(request.get());
}

CameraNode::~CameraNode()
{
  camera->requestCompleted.disconnect();
  if (camera->stop())
    std::cerr << "failed to stop camera" << std::endl;
  camera->release();
  camera_manager.stop();
}

void CameraNode::requestComplete(libcamera::Request *request)
{
  if (request->status() == libcamera::Request::RequestCancelled)
    return;

  assert(request->buffers().size() == 1);

  // get the stream and buffer from the request
  const libcamera::Stream *stream;
  libcamera::FrameBuffer *buffer;
  std::tie(stream, buffer) = *request->buffers().begin();

  const libcamera::FrameMetadata &metadata = buffer->metadata();

  // set time offset once for accurate timing using the device time
  if (time_offset == 0)
    time_offset = this->now().nanoseconds() - metadata.timestamp;

  // memory-map the frame buffer content
  assert(buffer->planes().size() == 1 && metadata.planes().size() == 1);
  const size_t buffer_size = metadata.planes()[0].bytesused;
  void *memory =
    mmap(NULL, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, buffer->planes()[0].fd.get(), 0);
  if (memory == MAP_FAILED)
    std::cerr << "mmap failed: " << std::strerror(errno) << std::endl;

  // send image data
  std_msgs::msg::Header hdr;
  hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
  hdr.frame_id = "camera";
  const libcamera::StreamConfiguration &cfg = stream->configuration();
  switch (cfg.pixelFormat) {
  case libcamera::formats::MJPEG:
  {
    // publish JPEG image
    sensor_msgs::msg::CompressedImage msg_img_jpeg;
    msg_img_jpeg.header = hdr;
    msg_img_jpeg.format = "jpeg";
    msg_img_jpeg.data.resize(buffer_size);
    memcpy(msg_img_jpeg.data.data(), memory, buffer_size);
    pub_image_compressed->publish(msg_img_jpeg);
    break;
  }
  case libcamera::formats::YUYV:
  {
    sensor_msgs::msg::Image msg_img;
    msg_img.header = hdr;
    msg_img.encoding = sensor_msgs::image_encodings::YUV422_YUY2;
    msg_img.width = cfg.size.width;
    msg_img.height = cfg.size.height;
    msg_img.step = cfg.stride;
    msg_img.data.resize(buffer_size);
    memcpy(msg_img.data.data(), memory, buffer_size);
    pub_image->publish(msg_img);
    break;
  }
  default:
    throw std::runtime_error("unsupported pixel format: " +
                             stream->configuration().pixelFormat.toString());
  }

  if (munmap(memory, buffer_size) == -1)
    std::cerr << "munmap failed: " << std::strerror(errno) << std::endl;

  // queue the request again for the next frame
  request->reuse(libcamera::Request::ReuseBuffers);
  camera->queueRequest(request);
}

} // namespace camera
