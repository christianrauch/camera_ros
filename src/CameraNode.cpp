#include "clamp.hpp"
#include "cv_to_pv.hpp"
#include "format_mapping.hpp"
#include "parameter_conflict_check.hpp"
#include "pv_to_cv.hpp"
#include "type_extent.hpp"
#include "types.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
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
  std::mutex request_lock;

  // timestamp offset (ns) from camera time to system time
  int64_t time_offset = 0;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image_compressed;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci;

  camera_info_manager::CameraInfoManager cim;

  OnSetParametersCallbackHandle::SharedPtr callback_parameter_change;

  // map parameter names to libcamera control id
  std::unordered_map<std::string, const libcamera::ControlId *> parameter_ids;
  // parameters that are to be set for every request
  std::unordered_map<unsigned int, libcamera::ControlValue> parameters;
  // keep track of set parameters
  ParameterMap parameters_full;
  std::mutex parameters_lock;

  void
  declareParameters();

  void
  requestComplete(libcamera::Request *request);

  rcl_interfaces::msg::SetParametersResult
  onParameterChange(const std::vector<rclcpp::Parameter> &parameters);

  void
  list_cameras();

  void
  list_stream_formats(const libcamera::StreamFormats &formats);

  void
  list_format_sizes(const libcamera::StreamConfiguration &configuration);
};

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)


struct buffer_t
{
  void *data;
  size_t size;
};


libcamera::StreamRole
get_role(const std::string &role)
{
  static const std::unordered_map<std::string, libcamera::StreamRole> roles_map = {
    {"raw", libcamera::StreamRole::Raw},
    {"still", libcamera::StreamRole::StillCapture},
    {"video", libcamera::StreamRole::VideoRecording},
    {"viewfinder", libcamera::StreamRole::Viewfinder},
  };

  try {
    return roles_map.at(role);
  }
  catch (const std::out_of_range &) {
    throw std::runtime_error("invalid stream role: \"" + role + "\"");
  }
}

CameraNode::CameraNode(const rclcpp::NodeOptions &options) : Node("camera", options), cim(this)
{
  // pixel format
  rcl_interfaces::msg::ParameterDescriptor param_descr_format;
  param_descr_format.description = "pixel format of streaming buffer";
  param_descr_format.read_only = true;
  declare_parameter<std::string>("format", {}, param_descr_format);

  // stream role
  rcl_interfaces::msg::ParameterDescriptor param_descr_role;
  param_descr_role.description = "stream role";
  param_descr_role.additional_constraints = "one of {raw, still, video, viewfinder}";
  param_descr_role.read_only = true;
  declare_parameter<std::string>("role", "video", param_descr_role);

  // image dimensions
  rcl_interfaces::msg::ParameterDescriptor param_descr_ro;
  param_descr_ro.read_only = true;
  declare_parameter<int64_t>("width", {}, param_descr_ro);
  declare_parameter<int64_t>("height", {}, param_descr_ro);

  // camera ID
  declare_parameter("camera", rclcpp::ParameterValue {}, param_descr_ro.set__dynamic_typing(true));

  // publisher for raw and compressed image
  pub_image = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", 1);
  pub_image_compressed =
    this->create_publisher<sensor_msgs::msg::CompressedImage>("~/image_raw/compressed", 1);
  pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 1);

  // start camera manager and check for cameras
  camera_manager.start();
  if (camera_manager.cameras().empty())
    throw std::runtime_error("no cameras available");

  // get the camera
  switch (get_parameter("camera").get_type()) {
  case rclcpp::ParameterType::PARAMETER_NOT_SET:
    // use first camera as default
    camera = camera_manager.cameras().front();
    list_cameras();
    RCLCPP_WARN_STREAM(get_logger(),
                       "no camera selected, using default: \"" << camera->id() << "\"");
    break;
  case rclcpp::ParameterType::PARAMETER_INTEGER:
  {
    const size_t id = get_parameter("camera").as_int();
    if (id >= camera_manager.cameras().size()) {
      list_cameras();
      throw std::runtime_error("camera with id " + std::to_string(id) + " does not exist");
    }
    camera = camera_manager.cameras().at(id);
    RCLCPP_DEBUG_STREAM(get_logger(), "found camera by id: " << id);
  } break;
  case rclcpp::ParameterType::PARAMETER_STRING:
  {
    const std::string name = get_parameter("camera").as_string();
    camera = camera_manager.get(name);
    if (!camera) {
      list_cameras();
      throw std::runtime_error("camera with name " + name + " does not exist");
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "found camera by name: \"" << name << "\"");
  } break;
  default:
    RCLCPP_ERROR_STREAM(get_logger(), "unuspported camera parameter type: "
                                        << get_parameter("camera").get_type_name());
    break;
  }

  if (!camera)
    throw std::runtime_error("failed to find camera");

  if (camera->acquire())
    throw std::runtime_error("failed to acquire camera");

  // configure camera stream
  std::unique_ptr<libcamera::CameraConfiguration> cfg =
    camera->generateConfiguration({get_role(get_parameter("role").as_string())});

  if (!cfg)
    throw std::runtime_error("failed to generate configuration");

  libcamera::StreamConfiguration &scfg = cfg->at(0);
  // store full list of stream formats
  const libcamera::StreamFormats &stream_formats = scfg.formats();
  const std::vector<libcamera::PixelFormat> &pixel_formats = scfg.formats().pixelformats();
  const std::string format = get_parameter("format").as_string();
  if (format.empty()) {
    list_stream_formats(stream_formats);
    // check if the default pixel format is supported
    if (format_type(scfg.pixelFormat) == FormatType::NONE) {
      // find first supported pixel format available by camera
      const auto result = std::find_if(
        pixel_formats.begin(), pixel_formats.end(),
        [](const libcamera::PixelFormat &fmt) { return format_type(fmt) != FormatType::NONE; });

      if (result == pixel_formats.end())
        throw std::runtime_error("camera does not provide any of the supported pixel formats");

      scfg.pixelFormat = *result;
    }

    RCLCPP_WARN_STREAM(get_logger(),
                       "no pixel format selected, using default: \"" << scfg.pixelFormat << "\"");
  }
  else {
    // get pixel format from provided string
    const libcamera::PixelFormat format_requested = libcamera::PixelFormat::fromString(format);
    if (!format_requested.isValid()) {
      list_stream_formats(stream_formats);
      throw std::runtime_error("invalid pixel format: \"" + format + "\"");
    }
    // check that requested format is supported by camera
    if (std::find(pixel_formats.begin(), pixel_formats.end(), format_requested) ==
        pixel_formats.end()) {
      list_stream_formats(stream_formats);
      throw std::runtime_error("pixel format \"" + format + "\" is unsupported by camera");
    }
    // check that requested format is supported by node
    if (format_type(format_requested) == FormatType::NONE)
      throw std::runtime_error("pixel format \"" + format + "\" is unsupported by node");
    scfg.pixelFormat = format_requested;
  }

  const libcamera::Size size(get_parameter("width").as_int(), get_parameter("height").as_int());
  if (size.isNull()) {
    list_format_sizes(scfg);
    scfg.size = scfg.formats().sizes(scfg.pixelFormat).back();
    RCLCPP_WARN_STREAM(get_logger(),
                       "no dimensions selected, auto-selecting: \"" << scfg.size << "\"");
  }
  else {
    scfg.size = size;
  }

  // store selected stream configuration
  const libcamera::StreamConfiguration selected_scfg = scfg;

  switch (cfg->validate()) {
  case libcamera::CameraConfiguration::Valid:
    break;
  case libcamera::CameraConfiguration::Adjusted:
    if (selected_scfg.pixelFormat != scfg.pixelFormat)
      list_stream_formats(stream_formats);
    if (selected_scfg.size != scfg.size)
      list_format_sizes(scfg);
    RCLCPP_WARN_STREAM(get_logger(), "stream configuration adjusted from \""
                                       << selected_scfg.toString() << "\" to \"" << scfg.toString()
                                       << "\"");
    break;
  case libcamera::CameraConfiguration::Invalid:
    throw std::runtime_error("failed to valid stream configurations");
    break;
  }

  if (camera->configure(cfg.get()) < 0)
    throw std::runtime_error("failed to configure streams");

  RCLCPP_INFO_STREAM(get_logger(), "camera \"" << camera->id() << "\" configured with "
                                               << scfg.toString() << " stream");

  set_parameter(rclcpp::Parameter("width", int64_t(scfg.size.width)));
  set_parameter(rclcpp::Parameter("height", int64_t(scfg.size.height)));
  set_parameter(rclcpp::Parameter("format", scfg.pixelFormat.toString()));

  // format camera name for calibration file
  const libcamera::ControlList &props = camera->properties();
  std::string cname = camera->id() + '_' + scfg.size.toString();
  const std::optional<std::string> model = props.get(libcamera::properties::Model);
  if (model)
    cname = model.value() + '_' + cname;

  // clean camera name of non-alphanumeric characters
  cname.erase(
    std::remove_if(cname.begin(), cname.end(), [](const char &x) { return std::isspace(x); }),
    cname.cend());
  std::replace_if(
    cname.begin(), cname.end(), [](const char &x) { return !std::isalnum(x); }, '_');

  if (!cim.setCameraName(cname))
    throw std::runtime_error("camera name must only contain alphanumeric characters");

  declareParameters();

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
  request_lock.lock();
  if (camera->stop())
    std::cerr << "failed to stop camera" << std::endl;
  request_lock.unlock();
  camera->release();
  camera_manager.stop();
}

void
CameraNode::declareParameters()
{
  // dynamic camera configuration
  ParameterMap parameters_init;
  for (const auto &[id, info] : camera->controls()) {
    // store control id with name
    parameter_ids[id->name()] = id;

    std::size_t extent;
    try {
      extent = get_extent(id);
    }
    catch (const std::runtime_error &e) {
      // ignore
      RCLCPP_WARN_STREAM(get_logger(), e.what());
      continue;
    }

    // format type description
    const std::string cv_descr =
      std::to_string(id->type()) + " " +
      std::string(extent > 1 ? "array[" + std::to_string(extent) + "]" : "scalar") + " range {" +
      info.min().toString() + "}..{" + info.max().toString() + "}" +
      (info.def().isNone() ? std::string {} : " (default: {" + info.def().toString() + "})");

    if (info.min().numElements() != info.max().numElements())
      throw std::runtime_error("minimum and maximum parameter array sizes do not match");

    // clamp default ControlValue to min/max range and cast ParameterValue
    const rclcpp::ParameterValue value =
      cv_to_pv(clamp(info.def(), info.min(), info.max()), extent);

    // get smallest bounds for minimum and maximum set
    rcl_interfaces::msg::IntegerRange range_int;
    rcl_interfaces::msg::FloatingPointRange range_float;

    switch (id->type()) {
    case libcamera::ControlTypeInteger32:
      range_int.from_value = max<libcamera::ControlTypeInteger32>(info.min());
      range_int.to_value = min<libcamera::ControlTypeInteger32>(info.max());
      break;
    case libcamera::ControlTypeInteger64:
      range_int.from_value = max<libcamera::ControlTypeInteger64>(info.min());
      range_int.to_value = min<libcamera::ControlTypeInteger64>(info.max());
      break;
    case libcamera::ControlTypeFloat:
      range_float.from_value = max<libcamera::ControlTypeFloat>(info.min());
      range_float.to_value = min<libcamera::ControlTypeFloat>(info.max());
      break;
    default:
      break;
    }

    rcl_interfaces::msg::ParameterDescriptor param_descr;
    param_descr.description = cv_descr;
    if (range_int.from_value != range_int.to_value)
      param_descr.integer_range = {range_int};
    if (range_float.from_value != range_float.to_value)
      param_descr.floating_point_range = {range_float};

    // declare parameters and set default or initial value
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "declare " << id->name() << " with default " << rclcpp::to_string(value));
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      declare_parameter(id->name(), cv_to_pv_type(id->type(), extent > 0), param_descr);
    }
    else {
      declare_parameter(id->name(), value, param_descr);
      parameters_init[id->name()] = value;
    }
  }

  // register callback to handle parameter changes
  // We have to register the callback after parameter declaration
  // to avoid callbacks interfering with the default parameter check.
  callback_parameter_change = add_on_set_parameters_callback(
    std::bind(&CameraNode::onParameterChange, this, std::placeholders::_1));

  // resolve conflicts of default libcamera configuration and user provided overrides
  std::vector<std::string> status;
  std::tie(parameters_init, status) =
    resolve_conflicts(parameters_init, get_node_parameters_interface()->get_parameter_overrides());

  for (const std::string &s : status)
    RCLCPP_WARN_STREAM(get_logger(), s);

  std::vector<rclcpp::Parameter> parameters_init_list;
  for (const auto &[name, value] : parameters_init)
    parameters_init_list.emplace_back(name, value);
  set_parameters(parameters_init_list);
}

void
CameraNode::requestComplete(libcamera::Request *request)
{
  request_lock.lock();

  if (request->status() == libcamera::Request::RequestComplete) {
    assert(request->buffers().size() == 1);

    // get the stream and buffer from the request
    const libcamera::Stream *stream;
    libcamera::FrameBuffer *buffer;
    std::tie(stream, buffer) = *request->buffers().begin();

    const libcamera::FrameMetadata &metadata = buffer->metadata();

    // set time offset once for accurate timing using the device time
    if (time_offset == 0)
      time_offset = this->now().nanoseconds() - metadata.timestamp;

    // memory-map the frame buffer planes
    assert(buffer->planes().size() == metadata.planes().size());
    std::vector<buffer_t> buffers;
    for (size_t i = 0; i < buffer->planes().size(); i++) {
      buffer_t mem;
      mem.size = metadata.planes()[i].bytesused;
      mem.data =
        mmap(NULL, mem.size, PROT_READ | PROT_WRITE, MAP_SHARED, buffer->planes()[i].fd.get(), 0);
      buffers.push_back(mem);
      if (mem.data == MAP_FAILED)
        std::cerr << "mmap failed: " << std::strerror(errno) << std::endl;
    }

    // send image data
    std_msgs::msg::Header hdr;
    hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
    hdr.frame_id = "camera";
    const libcamera::StreamConfiguration &cfg = stream->configuration();

    if (format_type(cfg.pixelFormat) == FormatType::RAW) {
      // raw uncompressed image
      assert(buffers.size() == 1);
      sensor_msgs::msg::Image::UniquePtr msg_img;
      msg_img = std::make_unique<sensor_msgs::msg::Image>();
      msg_img->header = hdr;
      msg_img->width = cfg.size.width;
      msg_img->height = cfg.size.height;
      msg_img->step = cfg.stride;
      msg_img->encoding = get_ros_encoding(cfg.pixelFormat);
      msg_img->is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__);
      msg_img->data.resize(buffers[0].size);
      memcpy(msg_img->data.data(), buffers[0].data, buffers[0].size);

      // compress to jpeg
      sensor_msgs::msg::CompressedImage::UniquePtr msg_img_compressed;
      msg_img_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      cv_bridge::toCvCopy(*msg_img)->toCompressedImageMsg(*msg_img_compressed);

      pub_image->publish(std::move(msg_img));
      pub_image_compressed->publish(std::move(msg_img_compressed));
    }
    else if (format_type(cfg.pixelFormat) == FormatType::COMPRESSED) {
      // compressed image
      assert(buffers.size() == 1);
      sensor_msgs::msg::CompressedImage::UniquePtr msg_img_compressed;
      msg_img_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg_img_compressed->header = hdr;
      msg_img_compressed->format = get_ros_encoding(cfg.pixelFormat);
      msg_img_compressed->data.resize(buffers[0].size);
      memcpy(msg_img_compressed->data.data(), buffers[0].data, buffers[0].size);

      // decompress into raw rgb8 image
      sensor_msgs::msg::Image::UniquePtr msg_img;
      msg_img = std::make_unique<sensor_msgs::msg::Image>();
      cv_bridge::toCvCopy(*msg_img_compressed, "rgb8")->toImageMsg(*msg_img);

      pub_image->publish(std::move(msg_img));
      pub_image_compressed->publish(std::move(msg_img_compressed));
    }
    else {
      throw std::runtime_error("unsupported pixel format: " +
                               stream->configuration().pixelFormat.toString());
    }

    for (const buffer_t &mem : buffers)
      if (munmap(mem.data, mem.size) == -1)
        std::cerr << "munmap failed: " << std::strerror(errno) << std::endl;

    sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
    ci.header = hdr;
    pub_ci->publish(ci);
  }
  else if (request->status() == libcamera::Request::RequestCancelled) {
    RCLCPP_ERROR_STREAM(get_logger(), "request '" << request->toString() << "' cancelled");
  }

  // queue the request again for the next frame
  request->reuse(libcamera::Request::ReuseBuffers);

  // update parameters
  parameters_lock.lock();
  for (const auto &[id, value] : parameters)
    request->controls().set(id, value);
  parameters.clear();
  parameters_lock.unlock();

  camera->queueRequest(request);

  request_lock.unlock();
}

rcl_interfaces::msg::SetParametersResult
CameraNode::onParameterChange(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  // check target parameter state (current and new parameters)
  // for conflicting configuration
  const std::vector<std::string> msgs = check_conflicts(parameters, parameters_full);
  if (!msgs.empty()) {
    result.successful = false;
    for (size_t i = 0; i < msgs.size(); i++) {
      if (msgs.size() > 1)
        result.reason += "(" + std::to_string(i) + ") ";
      result.reason += msgs[i];
      if (i < msgs.size() - 1)
        result.reason += "; ";
    }
    return result;
  }

  result.successful = true;

  for (const rclcpp::Parameter &parameter : parameters) {
    RCLCPP_DEBUG_STREAM(get_logger(), "setting " << parameter.get_type_name() << " parameter "
                                                 << parameter.get_name() << " to "
                                                 << parameter.value_to_string());

    if (parameter_ids.count(parameter.get_name())) {
      const libcamera::ControlId *id = parameter_ids.at(parameter.get_name());
      libcamera::ControlValue value = pv_to_cv(parameter, id->type());

      if (!value.isNone()) {
        // verify parameter type and dimension against default
        const libcamera::ControlInfo &ci = camera->controls().at(id);

        if (value.type() != id->type()) {
          result.successful = false;
          result.reason = parameter.get_name() + ": parameter types mismatch, expected '" +
                          std::to_string(id->type()) + "', got '" + std::to_string(value.type()) +
                          "'";
          return result;
        }

        const std::size_t extent = get_extent(id);
        if ((value.isArray() && (extent > 0)) && value.numElements() != extent) {
          result.successful = false;
          result.reason = parameter.get_name() + ": parameter dimensions mismatch, expected " +
                          std::to_string(extent) + ", got " + std::to_string(value.numElements());
          return result;
        }

        // check bounds and return error
        if (value < ci.min() || value > ci.max()) {
          result.successful = false;
          result.reason =
            "parameter value " + value.toString() + " outside of range: " + ci.toString();
          return result;
        }

        parameters_lock.lock();
        this->parameters[parameter_ids.at(parameter.get_name())->id()] = value;
        parameters_lock.unlock();

        parameters_full[parameter.get_name()] = parameter.get_parameter_value();
      }
    }
  }

  return result;
}

void
CameraNode::list_cameras()
{
  std::stringstream ss;
  ss << std::endl << ">> cameras:";
  for (size_t id = 0; id < camera_manager.cameras().size(); id++) {
    const std::shared_ptr<libcamera::Camera> camera = camera_manager.cameras().at(id);
    const std::string name =
      camera->properties().get(libcamera::properties::Model).value_or("UNDEFINED");
    ss << std::endl << "   " << id << ": " << name << " (" << camera->id() << ")";
  }
  RCLCPP_INFO_STREAM(get_logger(), ss.str());
}

void
CameraNode::list_stream_formats(const libcamera::StreamFormats &formats)
{
  // show supported pixel formats
  std::stringstream ss;
  ss << std::endl << ">> stream formats:";
  for (const libcamera::PixelFormat &pixelformat : formats.pixelformats()) {
    ss << std::endl
       << "   - Pixelformat: " << pixelformat.toString() << " ("
       << formats.range(pixelformat).min.toString() << " - "
       << formats.range(pixelformat).max.toString() << ")";
  }
  RCLCPP_INFO_STREAM(get_logger(), ss.str());
}

void
CameraNode::list_format_sizes(const libcamera::StreamConfiguration &configuration)
{
  std::stringstream ss;
  ss << std::endl << ">> " << configuration.pixelFormat << " format sizes:";
  for (const libcamera::Size &size : configuration.formats().sizes(configuration.pixelFormat))
    ss << std::endl << "   - " << size.toString();
  RCLCPP_INFO_STREAM(get_logger(), ss.str());
}

} // namespace camera
