---
title: micro-ROS Build System
permalink: /docs/concepts/build_system/
---

micro-ROS provides two ways of building a micro-ROS application for embedded platforms:
- _micro_ros_setup:_ integrates and hides the RTOS-specific build tools in few scripts provided as a ROS 2 package.
- _Platform-specific integrations:_ We have integrated micro-ROS with several platforms build tools. Click [here](/docs/concepts/build_system/external_build_systems/) to learn more.

**micro_ros_setup** provides a standalone build system in the form of a ROS 2 package for use in any normal ROS 2 workspace. This tool is available in the [micro-ROS/micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup) repository.

The **micro_ros_setup** tool allows compiling and generating images that contain micro-ROS apps for the [supported hardware](/docs/overview/hardware/) boards and [RTOSes](/docs/concepts/rtos/).

As the **micro_ros_setup** package can be installed like any other ROS 2 package, its usage will be through the ROS 2 CLI tool. Compiling, generating an image and flashing it on a board can be done just with four ROS 2 commands. A detailed description about the usage of this package can be found in the [tutorial section](/docs/tutorials/core/first_application_rtos/).

### micro-ROS client

Once installed, the build system tool provides some utilities that can be used in order to prepare, build, flash and use a micro-ROS application. The micro-ROS build system is a four-step procedure. In the first step, the user can create a new micro-ROS application by configuring the target hardware and RTOS:

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh [RTOS] [HARDWARE BOARD]
```

It is possible to obtain a list of the supported hardware by running the command without any argument. By doing so, it is possible to see that along with the RTOSes and hardware supported by micro-ROS this build system also provides with three extra options:
- By using `zephyr` as RTOS and `host` as hardware name, it is possible to obtain a Zephyr RTOS image with your micro-ROS app that runs in your host computer.
- By using just `host` as RTOS, micro-ROS will build a set of [micro-ROS demo applications](https://github.com/micro-ROS/micro-ROS-demos) natively in your host machine. These applications behave just like micro-ROS apps (using the same abstraction layers and middleware implementation) and allow the user to debug and test the applications on a PC.
- By using `generate_lib` as RTOS it is possible to configure the build system for generating static libraries (`.a`) and a set of headers (`include`) that can be linked in any other external tool. This option requires a valid CMake toolchain.

Once the build system has created the new firmware project, it is possible to configure it using:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
```

By running this command without any argument, it will output a list of example applications valid for the selected RTOS.
Common options available at this configuration step are:
  - `--transport` or `-t`: `udp`, `serial` or any hardware specific transport label
  - `--dev` or `-d`: agent string descriptor in a serial-like transport
  - `--ip` or `-i`: agent IP in a network-like transport
  - `--port` or `-p`: agent port in a network-like transport


Finally, it is possible to build and flash a micro-ROS app using:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh

# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```

### micro-ROS agent

The micro-ROS build system is also able to ease the compilation of the micro-ROS Agent in a ROS 2 workspace by using these commands:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent [OPTIONS]
```

**Tip 1:** To learn use of the micro_ros_setup build system hands-on, please see the [core tutorials](https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/).

**Tip 2 :** Remember that the micro-ROS Agent can be also be used with this simple Docker command: `docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO [OPTIONS]`
