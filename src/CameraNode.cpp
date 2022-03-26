#include "cast_cv.hpp"
#include "clamp.hpp"
#include "cv_to_pv.hpp"
#include "pv_to_cv.hpp"
#include "types.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>
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
  int64_t time_offset;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image_compressed;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_ci;

  camera_info_manager::CameraInfoManager cim;

  OnSetParametersCallbackHandle::SharedPtr callback_parameter_change;

  // map parameter names to libcamera control id
  std::unordered_map<std::string, const libcamera::ControlId *> parameter_ids;
  libcamera::ControlList parameters;
  std::mutex parameters_lock;

  void
  requestComplete(libcamera::Request *request);

  rcl_interfaces::msg::SetParametersResult
  onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CameraNode)


struct buffer_t
{
  void *data;
  size_t size;
};

// supported FourCC formats, without conversion
const std::unordered_map<uint32_t, std::string> map_format_raw = {
  // YUV encodings
  {libcamera::formats::YUYV.fourcc(), sensor_msgs::image_encodings::YUV422_YUY2},
  {libcamera::formats::YUV422.fourcc(), sensor_msgs::image_encodings::YUV422},
  // RAW encodings
  {libcamera::formats::SRGGB8.fourcc(), sensor_msgs::image_encodings::BAYER_RGGB8},
  {libcamera::formats::SGRBG8.fourcc(), sensor_msgs::image_encodings::BAYER_GRBG8},
  {libcamera::formats::SGBRG8.fourcc(), sensor_msgs::image_encodings::BAYER_GBRG8},
  {libcamera::formats::SBGGR8.fourcc(), sensor_msgs::image_encodings::BAYER_BGGR8},
  {libcamera::formats::SRGGB16.fourcc(), sensor_msgs::image_encodings::BAYER_RGGB16},
  {libcamera::formats::SGRBG16.fourcc(), sensor_msgs::image_encodings::BAYER_GRBG16},
  {libcamera::formats::SGBRG16.fourcc(), sensor_msgs::image_encodings::BAYER_GBRG16},
  {libcamera::formats::SBGGR16.fourcc(), sensor_msgs::image_encodings::BAYER_BGGR16},
};

// supported FourCC formats, without conversion, compressed
const std::unordered_map<uint32_t, std::string> map_format_compressed = {
  {libcamera::formats::MJPEG.fourcc(), "jpeg"},
};

CameraNode::CameraNode(const rclcpp::NodeOptions &options) : Node("camera", options), cim(this)
{
  // pixel format
  rcl_interfaces::msg::ParameterDescriptor param_descr_format;
  param_descr_format.description = "pixel format of streaming buffer";
  param_descr_format.read_only = true;
  declare_parameter<std::string>("format", {}, param_descr_format);

  // image dimensions
  rcl_interfaces::msg::ParameterDescriptor param_descr_ro;
  param_descr_ro.read_only = true;
  declare_parameter<int64_t>("width", {}, param_descr_ro);
  declare_parameter<int64_t>("height", {}, param_descr_ro);

  // camera ID
  declare_parameter<int64_t>("camera", 0, param_descr_ro);

  // publisher for raw and compressed image
  pub_image = this->create_publisher<sensor_msgs::msg::Image>("~/image_raw", 1);
  pub_image_compressed =
    this->create_publisher<sensor_msgs::msg::CompressedImage>("~/image_raw/compressed", 1);
  pub_ci = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 1);

  // start camera manager and check for cameras
  camera_manager.start();
  if (camera_manager.cameras().empty())
    throw std::runtime_error("no cameras available");

  std::cout << ">> cameras:" << std::endl;
  for (size_t id = 0; id < camera_manager.cameras().size(); id++) {
    const std::shared_ptr<libcamera::Camera> camera = camera_manager.cameras().at(id);
    const libcamera::ControlList &properties = camera->properties();
    const std::string name = properties.contains(libcamera::properties::Model)
                               ? properties.get(libcamera::properties::Model).value()
                               : "UNDEFINED";
    std::cout << id << ": " << name << " (" << camera->id() << ")" << std::endl;
  }

  // get the camera
  if (size_t(get_parameter("camera").as_int()) >= camera_manager.cameras().size())
    throw std::runtime_error("camera does not exist");
  camera = camera_manager.get(camera_manager.cameras().at(get_parameter("camera").as_int())->id());
  if (!camera)
    throw std::runtime_error("failed to find camera");

  if (camera->acquire())
    throw std::runtime_error("failed to acquire camera");

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
  std::string format;
  get_parameter("format", format);
  if (format.empty()) {
    // find first supported pixel format available by camera
    scfg.pixelFormat = {};
    for (const libcamera::PixelFormat &pixelformat : scfg.formats().pixelformats()) {
      if (map_format_raw.count(pixelformat.fourcc()) ||
          map_format_compressed.count(pixelformat.fourcc())) {
        scfg.pixelFormat = pixelformat;
        break;
      }
    }

    if (!scfg.pixelFormat.isValid())
      throw std::runtime_error("camera does not provide any of the supported pixel formats");
  }
  else {
    // get pixel format from provided string
    scfg.pixelFormat = libcamera::PixelFormat::fromString(format);
  }

  int64_t width = 0, height = 0;
  get_parameter("width", width);
  get_parameter("height", height);
  if (width)
    scfg.size.width = width;
  if (height)
    scfg.size.height = height;

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

  set_parameter(rclcpp::Parameter("width", int64_t(scfg.size.width)));
  set_parameter(rclcpp::Parameter("height", int64_t(scfg.size.height)));
  set_parameter(rclcpp::Parameter("format", scfg.pixelFormat.toString()));

  // format camera name for calibration file
  const libcamera::ControlList &props = camera->properties();
  std::string cname = camera->id() + '_' + scfg.size.toString();
  if (props.contains(libcamera::properties::Model))
    cname = props.get(libcamera::properties::Model).value() + '_' + cname;

  // clean camera name of non-alphanumeric characters
  cname.erase(
    std::remove_if(cname.begin(), cname.end(), [](const char &x) { return std::isspace(x); }),
    cname.cend());
  std::replace_if(
    cname.begin(), cname.end(), [](const char &x) { return !std::isalnum(x); }, '_');

  if (!cim.setCameraName(cname))
    throw std::runtime_error("camera name must only contain alphanumeric characters");

  //  // register callback to handle parameter changes
  //  callback_parameter_change = add_on_set_parameters_callback(
  //    std::bind(&CameraNode::onParameterChange, this, std::placeholders::_1));

  // dynamic camera configuration
  std::vector<rclcpp::Parameter> parameters_initial; // = options.parameter_overrides();
  // std::map<std::string, rclcpp::ParameterValue>
  //  const auto &initial_parameters = get_node_parameters_interface()->get_parameter_overrides();
  const auto &pp = get_node_parameters_interface()->get_parameter_overrides();
  for (const auto &[id, info] : camera->controls()) {
    // store control id with name
    parameter_ids[id->name()] = id;

    // cast all ControlValue to the type provided by the ControlId
    const libcamera::ControlValue val_def = cast_cv(info.def(), id->type());
    const libcamera::ControlValue val_min = cast_cv(info.min(), id->type());
    const libcamera::ControlValue val_max = cast_cv(info.max(), id->type());

    if (val_min.numElements() != val_max.numElements())
      throw std::runtime_error("minimum and maximum parameter array sizes do not match");

    // clamp default ControlValue to min/max range
    const libcamera::ControlValue val = clamp(val_def, val_min, val_max);

    // cast ControlValue to ParameterValue
    const rclcpp::ParameterValue value = cv_to_pv(val);

    // get smallest bounds for minimum and maximum set
    rcl_interfaces::msg::IntegerRange range_int;
    rcl_interfaces::msg::FloatingPointRange range_float;

    switch (id->type()) {
    case libcamera::ControlTypeInteger32:
      range_int.from_value = max<ControlTypeMap<libcamera::ControlTypeInteger32>::type>(val_min);
      range_int.to_value = min<ControlTypeMap<libcamera::ControlTypeInteger32>::type>(val_max);
      break;
    case libcamera::ControlTypeInteger64:
      range_int.from_value = max<ControlTypeMap<libcamera::ControlTypeInteger64>::type>(val_min);
      range_int.to_value = min<ControlTypeMap<libcamera::ControlTypeInteger64>::type>(val_max);
      break;
    case libcamera::ControlTypeFloat:
      range_float.from_value = max<ControlTypeMap<libcamera::ControlTypeFloat>::type>(val_min);
      range_float.to_value = min<ControlTypeMap<libcamera::ControlTypeFloat>::type>(val_max);
      break;
    default:
      break;
    }

    rcl_interfaces::msg::ParameterDescriptor param_descr;
    if (range_int.from_value != range_int.to_value)
      param_descr.integer_range = {range_int};
    if (range_float.from_value != range_float.to_value)
      param_descr.floating_point_range = {range_float};

    std::set<unsigned int> ignore {
      // interfers with AeEnable
      //      libcamera::controls::ExposureTime.id(),
    };

    //    const std::vector<rclcpp::Parameter> &ov = options.parameter_overrides();

    if (value.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      std::cout << "set " << id->name() << ", def: " << rclcpp::to_string(value) << std::endl;
      declare_parameter(id->name(), value, param_descr, true);
      //      declare_parameter(id->name(), value, param_descr);
      // setting the ExposureTime parameter right at the beginning causes:
      //   ERROR V4L2 [...]: Unable to set controls: Invalid argument
      //   ERROR UVC [...] Failed to set controls: -22
      //      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr pi = this->get_node_parameters_interface();

      std::cout << "param: " << get_parameter(id->name()) << std::endl;
      if (pp.count(id->name()))
        parameters_initial.emplace_back(rclcpp::Parameter(id->name(), pp.at(id->name())));
      else
        parameters_initial.emplace_back(rclcpp::Parameter(id->name(), value));
      //      for (parameters_initial)
      //      if (!ignore.count(id->id()))
      //        parameters_initial.push_back(get_parameter(id->name()));
      // TODO: handle conflicting types
      // TODO: handle limits
    }
  }

  // set initial parameters
  // TODO: find conflicting parameters
  onParameterChange(parameters_initial);
  //  onParameterChange(options.parameter_overrides()); // parameter_overrides is empty

  //  auto bla1 = options.parameter_overrides();
  //  auto bla2 = this->get_node_parameters_interface()->get_parameter_overrides();

  // register callback to handle parameter changes
  callback_parameter_change = add_on_set_parameters_callback(
    std::bind(&CameraNode::onParameterChange, this, std::placeholders::_1));

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
  if (camera->stop())
    std::cerr << "failed to stop camera" << std::endl;
  camera->requestCompleted.disconnect();
  camera->release();
  camera_manager.stop();
}

void
CameraNode::requestComplete(libcamera::Request *request)
{
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
    }

    // send image data
    std_msgs::msg::Header hdr;
    hdr.stamp = rclcpp::Time(time_offset + int64_t(metadata.timestamp));
    hdr.frame_id = "camera";
    const libcamera::StreamConfiguration &cfg = stream->configuration();

    if (map_format_raw.count(cfg.pixelFormat.fourcc())) {
      // raw uncompressed image
      assert(buffers.size() == 1);
      sensor_msgs::msg::Image::UniquePtr msg_img;
      msg_img = std::make_unique<sensor_msgs::msg::Image>();
      msg_img->header = hdr;
      msg_img->width = cfg.size.width;
      msg_img->height = cfg.size.height;
      msg_img->step = cfg.stride;
      msg_img->encoding = map_format_raw.at(cfg.pixelFormat.fourcc());
      msg_img->data.resize(buffers[0].size);
      memcpy(msg_img->data.data(), buffers[0].data, buffers[0].size);

      // compress to jpeg
      sensor_msgs::msg::CompressedImage::UniquePtr msg_img_compressed;
      msg_img_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      cv_bridge::toCvCopy(*msg_img)->toCompressedImageMsg(*msg_img_compressed);

      pub_image->publish(std::move(msg_img));
      pub_image_compressed->publish(std::move(msg_img_compressed));
    }
    else if (map_format_compressed.count(cfg.pixelFormat.fourcc())) {
      // compressed image
      assert(buffers.size() == 1);
      sensor_msgs::msg::CompressedImage::UniquePtr msg_img_compressed;
      msg_img_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg_img_compressed->header = hdr;
      msg_img_compressed->format = map_format_compressed.at(cfg.pixelFormat.fourcc());
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

    pub_ci->publish(cim.getCameraInfo());
    //    const sensor_msgs::msg::CameraInfo ci = cim.getCameraInfo();
    //    std::cout << std::to_string(ci) << std::endl;
    //    rosidl_typesupport_introspection_cpp::
    //    ci.
    //    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> ser;
    //    ser.serialize_message(,ci);
    //    rcl_params_t *lll = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
    //    std::cout << rosidl_generator_traits::to_yaml(ci) << std::endl;
  }
  else if (request->status() == libcamera::Request::RequestCancelled) {
    RCLCPP_ERROR_STREAM(get_logger(), "request '" << request->toString() << "' cancelled");
  }

  // queue the request again for the next frame
  request->reuse(libcamera::Request::ReuseBuffers);

  // update parameters
  parameters_lock.lock();
  request->controls() = parameters;
  parameters.clear();
  parameters_lock.unlock();

  camera->queueRequest(request);
}

rcl_interfaces::msg::SetParametersResult
CameraNode::onParameterChange(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter &parameter : parameters) {
    std::cout << "set " << parameter.get_name() << ": " << parameter.value_to_string() << " ("
              << parameter.get_type_name() << ")" << std::endl;

    if (parameter_ids.count(parameter.get_name())) {
      const libcamera::ControlId *id = parameter_ids.at(parameter.get_name());
      libcamera::ControlValue value = pv_to_cv(parameter, id->type());

      //      switch (parameter.get_type()) {
      //      case rclcpp::ParameterType::PARAMETER_NOT_SET:
      //        break;
      //      case rclcpp::ParameterType::PARAMETER_BOOL:
      //        value.set(parameter.as_bool());
      //        break;
      //      case rclcpp::ParameterType::PARAMETER_INTEGER:
      //        if (id->type() == libcamera::ControlTypeInteger32)
      //          value.set(CTInteger32(parameter.as_int()));
      //        else if (id->type() == libcamera::ControlTypeInteger64)
      //          value.set(CTInteger64(parameter.as_int()));
      //        else {
      //          result.successful = false;
      //          result.reason =
      //            parameter.get_name() + ": invalid integer type: " + std::to_string(id->type());
      //          return result;
      //        }
      //        break;
      //      case rclcpp::ParameterType::PARAMETER_DOUBLE:
      //        value.set(CTFloat(parameter.as_double()));
      //        break;
      //      case rclcpp::ParameterType::PARAMETER_STRING:
      //        value.set(parameter.as_string());
      //        break;
      //      case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      //        value.set(libcamera::Span<const CTByte>(parameter.as_byte_array()));
      //        break;
      //      case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      //        result.successful = false;
      //        result.reason = parameter.get_name() + ": unsupported type: " + parameter.get_type_name();
      //        return result;
      //      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      //        value.set(libcamera::Span<const CTInteger64>(parameter.as_integer_array()));
      //        break;
      //      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      //      {
      //        // convert to float vector
      //        const std::vector<double> &vd = parameter.as_double_array();
      //        const std::vector<CTFloat> vf(vd.begin(), vd.end());
      //        value.set(libcamera::Span<const CTFloat>(vf));
      //        break;
      //      }
      //      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      //        value.set(libcamera::Span<const CTString>(parameter.as_string_array()));
      //        break;
      //      }

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

        if (value.isArray() != ci.def().isArray() || value.numElements() != ci.def().numElements())
        {
          result.successful = false;
          result.reason = parameter.get_name() + ": parameter dimensions mismatch, expected " +
                          std::to_string(ci.def().numElements()) + ", got " +
                          std::to_string(value.numElements());
          return result;
        }

        //        // check bounds and return error
        //        if (value < ci.min() || value > ci.max()) {
        //          result.successful = false;
        //          result.reason =
        //            "parameter value " + value.toString() + " outside of range: " + ci.toString();
        //          return result;
        //        }

        parameters_lock.lock();
        std::cout << "set cam: " << value.toString() << std::endl;
        this->parameters.set(parameter_ids.at(parameter.get_name())->id(), value);
        //        libcamera::ControlValidator val;
        //        libcamera::controls::controls;
        // TODO: use 'ControlValidator' for ControlList parameters ?

        //        // exposure -> disable AE
        //        if (parameter_ids.at(parameter.get_name())->id() ==
        //              libcamera::controls::ExposureTime.id() &&
        //            this->parameters.contains(libcamera::controls::AeEnable) &&
        //            this->parameters.get(libcamera::controls::AeEnable))
        //        {
        //          this->parameters.set(libcamera::controls::AeEnable, false);
        //        }

        //        // AE -> remove exposure
        //        if (parameter_ids.at(parameter.get_name())->id() == libcamera::controls::AeEnable.id() &&
        //            parameter.as_bool() && this->parameters.contains(libcamera::controls::ExposureTime))
        //        {
        //          // TODO: remove exposure
        //          //          this->parameters.idMap()->at(libcamera::controls::ExposureTime.id());
        //          //          this->parameters = libcamera::controls::controls; // ??
        //          //          this->parameters.set(libcamera::controls::ExposureTime, int32_t {});
        //        }

        parameters_lock.unlock();
      }
    }
  }

  return result;
}

} // namespace camera
