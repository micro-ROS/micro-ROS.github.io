---
title: External Build Systems
permalink: /docs/concepts/build_system/external_build_systems/
---

Once you have read about the oficial[ **micro_ros_setup** tool](/docs/concepts/build_system/), this page will present some other approaches for building micro-ROS as a module or component integrated in other build systems.

## micro-ROS component for ESP-IDF

The [micro-ROS component for ESP-IDF](https://github.com/micro-ROS/micro_ros_espidf_component) allows to integrate micro-ROS as a component in an Espressif ESP-IDF Build System. This component allows the user to integrate the micro-ROS API and utilities in an already created ESP-IDF project just by cloning or copying a folder. 

Configuration of the micro-ROS library is based on the `colcon.meta` file. For more details visit the [Github repository](https://github.com/micro-ROS/micro_ros_espidf_component).

## micro-ROS module for Zephyr

The [micro-ROS module for Zephyr](https://github.com/micro-ROS/micro_ros_zephyr_module) allows to integrate micro-ROS as a module in a Zephyr RTOS project. This component allows the user to integrate the micro-ROS API and utilities in an already created Zephyr RTOS project just by cloning or copying a folder.

The procedure for configuring the built micro-ROS library is based in `colcon.meta`. For more details visit the [Github repository](https://github.com/micro-ROS/micro_ros_espidf_component).

## micro-ROS for Arduino

The [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino) support package is a special port of micro-ROS provided as a set of precompiled libraries for specific platforms. The main reason of this approach is that Arduino does not allow the build of a complex library such as micro-ROS, so by using this solution a ready-to-use solution is provided to the Arduino users.

Along with this support package there are [detailed instructions](https://github.com/micro-ROS/micro_ros_arduino#how-to-build-the-precompiled-library) for rebuilding the micro-ROS for Arduino libraries for users that need to tune the default configuration.
