# ROS 2 node for libcamera

This ROS 2 node provides support for a variety of cameras via [libcamera](https://libcamera.org). Amongst others, this node supports V4L2 and [Raspberry Pi cameras](https://www.raspberrypi.com/documentation/computers/camera_software.html).


## Installation

Binary packages are available via the ROS package repository for some Linux and ROS distributions (check with `rosdep resolve camera_ros`). If it's available, you can install the DEB or RPM packages via:
```sh
# source a ROS distribution
source /opt/ros/$ROS_DISTRO/setup.bash
# DEB package
sudo apt install ros-$ROS_DISTRO-camera-ros
# RPM package
sudo dnf install ros-$ROS_DISTRO-camera-ros
```

> [!NOTE]
> This also installs the package [`libcamera`](https://index.ros.org/r/libcamera/) as dependency. This is the bloomed version of the official upstream repo at https://git.libcamera.org/libcamera/libcamera.git and may not contain full support for all Raspberry Pi camera modules. If you need full camera module support on Raspberry Pi, you have to build the "raspberrypi" fork from https://github.com/raspberrypi/libcamera manually.


## Build Instructions

### libcamera

The `camera_ros` node depends on libcamera version 0.1 or later. There are different ways to install this dependency:

- __System Package:__ Most Linux distributions provide a binary libcamera package. However, the Raspberry Pi OS uses a [custom libcamera fork](https://github.com/raspberrypi/libcamera) with additional support for newer camera modules. When using the distribution packages, you have to skip the `libcamera` rosdep key when resolving dependencies (`rosdep install [...] --skip-keys=libcamera`).

- __ROS Package:__ You can also install a newer version from the ROS repo (package `ros-$ROS_DISTRO-libcamera`). This package will be installed by default when building `camera_ros` from source and resolving the rosdep keys.

- __Source:__ Finally, you can always build libcamera from source. This is currently the only option for using the "raspberrypi" fork on Ubuntu. You can build libcamera as part of the ROS workspace using `colcon-meson`. This is recommended over a system-wide installation as it avoids conflicts with the system package. You will need to install the following dependencies:
    1. Install the libcamera build dependencies according to https://libcamera.org/getting-started.html#dependencies.
    2. Install `colcon-meson` via the package manager, `sudo apt install -y python3-colcon-meson`, or pip, `pip install colcon-meson`.


### build camera_ros

The `camera_ros` package is built in a colcon workspace. The following instructions assume that you are building libcamera from source in the colcon workspace.

```sh
# create workspace with camera_ros package
mkdir -p ~/camera_ws/
cd ~/camera_ws/
git clone https://github.com/christianrauch/camera_ros.git src/camera_ros

# optional: build libcamera in colcon workspace
pip install colcon-meson
git clone https://git.libcamera.org/libcamera/libcamera.git src/libcamera

# resolve binary dependencies and build workspace
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --skip-keys=libcamera
colcon build
```

If you installed libcamera externally, you can omit the `colcon-meson` and `libcamera` steps. Additionally, if there is a binary package and a rosdep entry for libcamera (check with `rosdep resolve libcamera`) you can also omit `--skip-keys=libcamera` and have this binary dependency resolved automatically.


## Launching the Node

The package provides a standalone node executable:
```sh
ros2 run camera_ros camera_node
```
a composable node (`camera::CameraNode`):
```sh
ros2 component standalone camera_ros camera::CameraNode
```
and an example launch file for the composable node:
```sh
ros2 launch camera_ros camera.launch.py
```


## Interfaces

The camera node interfaces are compatible with the [`image_pipeline`](https://github.com/ros-perception/image_pipeline) stack. The node publishes the camera images and camera parameters and provides a service to set the camera parameters.

### Topics

| name                     | type                              | description        |
| ------------------------ | --------------------------------- | ------------------ |
| `~/image_raw`            | `sensor_msgs/msg/Image`           | image              |
| `~/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | image (compressed) |
| `~/camera_info`          | `sensor_msgs/msg/CameraInfo`      | camera parameters  |

### Services

| name                | type                            | description           |
| ------------------- | ------------------------------- | --------------------- |
| `~/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | set camera parameters |


## Parameters

The node provides two sets of parameters:
1. static read-only parameters to configure the camera stream once at the beginning
2. dynamic parameters which are generated from the camera controls to change per-frame settings and which can be changed at runtime

Those parameters can be set on the command line:
```sh
# standalone executable
ros2 run camera_ros camera_node --ros-args -p param1:=arg1 -p param2:=arg2
# composable node
ros2 component standalone camera_ros camera::CameraNode -p param1:=arg1 -p param2:=arg2
```
or dynamically via the [ROS parameter API](https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html).

### Static Camera Stream Configuration

The camera stream is configured once when the node starts via the following static read-only parameters:

| name              | type                  | description                                                                                                                 |
| ----------------- | --------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `camera`          | `integer` or `string` | selects the camera by index (e.g. `0`) or by name (e.g. `/base/soc/i2c0mux/i2c@1/ov5647@36`) [default: `0`]                 |
| `role`            | `string`              | configures the camera with a `StreamRole` (possible choices: `raw`, `still`, `video`, `viewfinder`) [default: `viewfinder`] |
| `format`          | `string`              | a `PixelFormat` that is supported by the camera [default: auto]                                                             |
| `width`, `height` | `integer`             | desired image resolution [default: auto]                                                                                    |


The configuration is done in the following order:
1. select camera via `camera`
2. configure camera stream via `role`
3. set the pixel format for the stream via `format`
4. set the image resolution for the stream via `width` and `height`

Each stream role only supports a discrete set of data stream configurations as a combination of the image resolution and pixel format. The selected stream configuration is validated at the end of this sequence and adjusted to the closest valid stream configuration.

By default, the node will select the first available camera and configures it with the default pixel format and image resolution. If a parameter has not been set, the node will print the available options and inform the user about the default choice.

The node avoids memory copies of the image data by directly mapping from a camera pixel format to a ROS image format, with the exception of converting between "raw" and "compressed" image formats when requested by the user. As an effect, not all pixel formats that are supported by the camera may be supported by the ROS image message. Hence, the options for `format` are limited to pixel formats that are supported by the camera and the raw ROS image message.

### Dynamic Frame Configuration (controls)

The dynamic parameters are created at runtime by inspecting the [controls](https://libcamera.org/api-html/namespacelibcamera_1_1controls.html) that are exposed by the camera. The set of exposed controls is camera-dependent. The ROS parameters are directly formatted from the exposed controls, hence the parameter names match the control names.

libcamera does not expose the framerate directly as a parameter. Instead, the framerate range has to be converted to a duration:
$$\text{duration} = \frac{1}{\text{framerate}} \cdot 10^6 \\ µs$$
and then set via the control `FrameDurationLimits`, if it is exposed by the camera:
```sh
# set fixed framerate of 20 Hz (50 ms)
ros2 run camera_ros camera_node --ros-args -p FrameDurationLimits:="[50000,50000]"
```


## Calibration

The node uses the `CameraInfoManager` to manage the [camera parameters](https://docs.ros.org/en/rolling/p/image_pipeline/camera_info.html), such as the camera intrinsics for projection and distortion coefficients for rectification. Its tasks include loading the parameters from the calibration file `~/.ros/camera_info/$NAME.yaml`, publishing them on `~/camera_info` and setting new parameters via service `~/set_camera_info`.

If the camera has not been calibrated yet and the calibration file does not exist, the node will warn the user about the missing file (`Camera calibration file ~/.ros/camera_info/$NAME.yaml not found`) and publish zero-initialised intrinsic parameters. If you do not need to project between the 2D image plane and the 3D camera frame or rectify the image, you can safely ignore this.

To calibrate the camera and set the parameters, you can use the [`cameracalibrator`](https://docs.ros.org/en/rolling/p/camera_calibration/) from the `camera_calibration` package or any other node that interfaces with the `~/set_camera_info` service.


## Trouble Shooting

To debug the node, first compile it in `Debug` mode:
```sh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```
and then run the node with libcamera and ROS debug information in `gdb`:
```sh
LIBCAMERA_LOG_LEVELS=*:DEBUG ros2 run --prefix 'gdb -ex run --args' camera_ros camera_node --ros-args --log-level debug -p width:=640 -p height:=480
```
