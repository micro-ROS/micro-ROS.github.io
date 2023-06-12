---
title: External Build Systems
permalink: /docs/concepts/build_system/external_build_systems/
---

Once you have read about the official [**micro_ros_setup** tool](/docs/concepts/build_system/), this page will present some other approaches for building micro-ROS as a module or component integrated into other build systems.

## micro-ROS component for ESP-IDF

The [micro-ROS component for ESP-IDF](https://github.com/micro-ROS/micro_ros_espidf_component) allows to integrate micro-ROS as a component in an Espressif ESP-IDF Build System. This component allows the user to integrate the micro-ROS API and utilities in an already created ESP-IDF project just by cloning or copying a folder.

Configuration of the micro-ROS library is based on the `colcon.meta` file. For more details visit the [Git repository](https://github.com/micro-ROS/micro_ros_espidf_component).

## micro-ROS module for Zephyr

The [micro-ROS module for Zephyr](https://github.com/micro-ROS/micro_ros_zephyr_module) allows to integrate micro-ROS as a module in a Zephyr-based project. In detail, it enables to integrate the micro-ROS API and utilities in an existing Zephyr project just by cloning or copying a folder.

The procedure for configuring the built micro-ROS library is based in `colcon.meta`. For more details visit the [Git repository](https://github.com/micro-ROS/micro_ros_espidf_component).

## micro-ROS for Arduino

The [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino) support package is a special port of micro-ROS provided as a set of precompiled libraries for specific platforms. The main reason for this approach is that Arduino does not allow the build of a complex library such as micro-ROS, so by using this approach a ready-to-use solution is provided to the Arduino users.

Along with this support package, there are [detailed instructions](https://github.com/micro-ROS/micro_ros_arduino#how-to-build-the-precompiled-library) for rebuilding the micro-ROS for Arduino libraries for users that need to tune the default configuration.

## micro-ROS for STM32CubeMX

The [micro-ROS for STM32CubeMX](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils) package is a set of utilities which enables the seamless configuration, set-up and integration of micro-ROS into an STM32 controller based project. As such, it allows micro-ROS to be virtually supported by the full set of boards offered by <a href="https://www.st.com/content/st_com/en.html">STMicroelectronics</a>.

Its usage is based on Dockers, via a prepared [Dockerfile](https://github.com/micro-ROS/docker/blob/humble/micro-ROS-static-library-builder/Dockerfile) which eases micro-ROS library generation outside of a ROS 2 environment.
