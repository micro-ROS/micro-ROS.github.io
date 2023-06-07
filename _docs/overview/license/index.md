---
title: License Overview
permalink: /docs/overview/license/
redirect_from:
  - /license/
---

This page gives a coarse overview to the license situation of micro-ROS. It is not intended as a legal advice and it does not relieve you to look at the license texts of all components that are used by your micro-ROS-based application or system (i.e. “product”).

The following architecture diagram lists the most important repositories with their respective licenses in the form `GitHubOrganization/RepositioryName [LicenseName]`.

<img src="/img/micro-ROS_license_overview.png" style="display: block; margin: auto; width: 100%; max-width: 650px;"/>

All packages taken from standard ROS 2 come under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0). Similarly, all middleware- and client-library-related packages that have been created in the micro-ROS project are provided under Apache 2.0. In the same way, eProsima’s implementation of the DDS-XRCE standard named Micro XRCE-DDS is provided under Apache 2.0.

One exception is the micro-ROS benchmarking tool. As it links against GPL v3 licensed libraries, it is itself provided under GPL v3. As long as the tool is used by you during development only but not included in the distributed product, this does not affect the license of your product.

Things get particularly interesting at the RTOS level: The micro-ROS build tool micro_ros_setup and the various modules for external build systems are provided under Apache 2.0 but use or are combined with very differently licensed RTOS and board support components. The fact, that typical embedded toolchains build the whole software (RTOS, micro-ROS and application) into one binary image makes the situation more complex compared to typical desktop operating systems with clear separation of individual executables and the OS kernel (cf. for example the Linux syscall exception to GPL).

We are aware of the following important license specifics in the RTOS supported by micro-ROS:

* NuttX license clearing: With the incubation at The Apache Software Foundation in December 2019, there has been significant license cleanup work. The [changelog for version 10.1](https://cwiki.apache.org/confluence/display/NUTTX/NuttX+10.1) states that thousands of NuttX files have been converted (from BSD) to Apache 2.0 and that the listing of 3rd party licenses used in NuttX has been improved.
* NuttX and uClibc++: Before NuttX version 10, micro-ROS on NuttX required the use of the LGPL-licensed uClibc++ library.
* ST-specific extensions for FreeRTOS: The [micro-ROS/freertos_apps](https://github.com/micro-ROS/freertos_apps/) repository contains extensions for various microcontroller families. Some of the header files for microcontrollers by STMicroelectronics are provided under ST's [Ultimate Liberty license](https://www.st.com/SLA0044), which "must be used and execute solely and exclusively on or in combination with a microcontroller or microprocessor device manufactured by or for STMicroelectronics."
* Third-party licenses in Arm® Mbed™ OS: The licenses of the third-party components are listed in the [LICENSE.md file](https://github.com/ARMmbed/mbed-os/blob/master/LICENSE.md) in the root of the repository.

... and in the corresponding tooling:

* GPL-licensed build scripts in Zepyhr: The third-party licenses are given directly [in the source tree](https://github.com/zephyrproject-rtos/zephyr/), but [docs.zephyrproject.org/latest/LICENSING.html](https://docs.zephyrproject.org/latest/LICENSING.html) states explicitly that few build scripts are used under GPL v2.
* GPL-licensed build tool files in ESP-IDF: The Espressif IoT Development Framework used for the ESP32 includes files menuconfig (Kconfig) and several other build tooling files licensed under GPL v2 or v3.
* Static library for Arduino IDE: The [micro_ros_arduino repository](https://github.com/micro-ROS/micro_ros_arduino) provides a static library `libmicroros.a` of the micro-ROS stack for use with the Arduino IDE. In detail, multiple versions of this library are provided, built for different microcontroller families using suitable cross-compiler configurations. The list of repositories included in the library can be found in the [`built_packages`](https://github.com/micro-ROS/micro_ros_arduino/blob/iron/built_packages) file in the root of the repository.