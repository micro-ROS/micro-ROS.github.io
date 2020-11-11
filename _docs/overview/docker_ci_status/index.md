---
title: Docker and CI status
permalink: /docs/overview/docker_ci_status/
---

## Docker Images Status

| Image | Description | Status
-|-|-:
| base  | Base image with ROSDISTRO installation + micro-ROS specific build system. Used as base of any other micro-ROS images | [![Docker Automated build](https://img.shields.io/docker/cloud/automated/microros/base.svg?logo=docker)](https://hub.docker.com/r/microros/base/)[![Docker Build Status](https://img.shields.io/docker/cloud/build/microros/base.svg?logo=docker)](https://hub.docker.com/r/microros/base/)[![Compare Images](https://images.microbadger.com/badges/image/microros/base.svg)](https://microbadger.com/images/microros/base)
| micro-ros-agent | Image containing a pre-compiled micro-ROS-Agent, ready to use as standalone application | [![Docker Automated build](https://img.shields.io/docker/cloud/automated/microros/micro-ros-agent.svg?logo=docker)](https://hub.docker.com/r/microros/micro-ros-agent/)[![Docker Build Status](https://img.shields.io/docker/cloud/build/microros/micro-ros-agent.svg?logo=docker)](https://hub.docker.com/r/microros/micro-ros-agent/)[![Compare Images](https://images.microbadger.com/badges/image/microros/micro-ros-agent.svg)](https://microbadger.com/images/microros/micro-ros-agent)
| micro-ros-demos | Contains precompiled micro-ROS-demos, ready to use to view micro-ROS funcionality | [![Docker Automated build](https://img.shields.io/docker/cloud/automated/microros/micro-ros-demos.svg?logo=docker)](https://hub.docker.com/r/microros/micro-ros-demos/)[![Docker Build Status](https://img.shields.io/docker/cloud/build/microros/micro-ros-demos.svg?logo=docker)](https://hub.docker.com/r/microros/micro-ros-demos/)[![Compare Images](https://images.microbadger.com/badges/image/microros/micro-ros-demos.svg)](https://microbadger.com/images/microros/micro-ros-demos)

## CI Status for Tooling and Website

<!--Add future CI test-->

| Repository | Description | Status
-|-|-:
|[micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup)| Micro-ROS tool to build and flash Micro-ROS to every supported platform.|[![GitHub Actions status](https://github.com/micro-ROS/micro_ros_setup/workflows/CI/badge.svg)](https://github.com/micro-ROS/micro_ros_setup/actions)
|[micro-ROS.github.io](https://github.com/micro-ROS/micro-ROS.github.io)| Micro-ROS official webpage repository.|[![GitHub Actions status](https://github.com/micro-ROS/micro-ROS.github.io/workflows/CI/badge.svg)](https://github.com/micro-ROS/micro-ROS.github.io/actions)

## Source Code Repositories

Major repositories in order of the layers are:

* Applications:
  * Kobuki demo: [embedded robot driver](https://github.com/micro-ROS/nuttx_apps/tree/kobuki_rcl_port/examples/kobuki) and [remote ROS 2 software](https://github.com/micro-ROS/micro-ROS_kobuki_demo)
* Client library:
  * Extensions to rcl, i.e. the ROS 2 C API: [rcl_executor](https://github.com/micro-ROS/rcl_executor), ...
  * Extensions for rclcpp: [system_modes](https://github.com/micro-ROS/system_modes/), [TF improvements](https://github.com/micro-ROS/geometry2), ...
* Middleware:
  * eProsima's open-source implementation of DDS-XRCE: [Micro-XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS)
  * RMW adapter for Micro-XRCE-DDS: [rmw-microxrcedds](https://github.com/micro-ROS/rmw-microxrcedds)
  * Type support for Micro-XRCE-DDS: [rosidl_typesupport_microxrcedds](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds)
  * Agent (bridge) to ROS 2: [micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent)
* RTOS:
  * Our [NuttX fork](https://github.com/micro-ROS/NuttX), but most additions were contributed back.
  * Example applications for NuttX directly are in [nuttx_apps](https://github.com/micro-ROS/nuttx_apps)

Most repositories can be found in GitHub's micro-ROS organization at [github.com/micro-ROS/](https://github.com/micro-ROS/).

### List of Repositories

| Name                            | Documentation | Release | CI | Issues |
|:--------------------------------|:--------------|:--------|:---|:-------|
| rmw-microxrcedds                | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/rmw-microxrcedds/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/micro-ROS/rmw-microxrcedds/tree/release-crystal-20190312) | | [![](https://img.shields.io/github/issues/micro-ROS/rmw-microxrcedds)](https://github.com/micro-ROS/rmw-microxrcedds/issues) |
| rosidl_typesupport_microxrcedds |  [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds/blob/release-crystal-20190312/README.md) |   | [![](https://img.shields.io/github/issues/micro-ROS/rosidl_typesupport_microxrcedds)](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds/issues) |
| micro-ROS-Agent                 |  [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/micro-ROS-Agent/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/micro-ROS/micro-ROS-Agent/blob/release-crystal-20190312/README.md) |  [![](http://build.ros2.org/buildStatus/icon?job=Cbin_uB64__micro-xrce-dds-agent__ubuntu_bionic_amd64__binary)](https://github.com/micro-ROS/micro-ROS-doc/blob/crystal/Installation/repos/agent_minimum.repos) | [![](https://img.shields.io/github/issues/micro-ROS/micro-ROS-Agent)](https://github.com/micro-ROS/micro-ROS-Agent/issues) |
| Micro XRCE-DDS                  | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://micro-xrce-dds.readthedocs.io/en/latest/) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/eProsima/Micro-XRCE-DDS/tree/v1.0.3) [![](https://img.shields.io/badge/ROS-dashing-brightgreen)](https://github.com/eProsima/Micro-XRCE-DDS/tree/v1.1.0) |    | [![](https://img.shields.io/github/issues/eProsima/Micro-XRCE-DDS.svg)](https://github.com/eProsima/Micro-XRCE-DDS/issues) |
| system_modes                    | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/system_modes/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-dashing-brightgreen)](https://github.com/micro-ROS/system_modes/releases) [![](https://img.shields.io/badge/ROS-eloquent-brightgreen)](https://github.com/micro-ROS/system_modes/releases)[![](https://img.shields.io/badge/ROS-foxy-brightgreen)](https://github.com/micro-ROS/system_modes/releases) | [![Build Status](http://build.ros2.org/job/Dbin_uB64__system_modes__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros2.org/job/Dbin_uB64__system_modes__ubuntu_bionic_amd64__binary/) [![Build Status](http://build.ros2.org/job/Ebin_uB64__system_modes__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros2.org/job/Ebin_uB64__system_modes__ubuntu_bionic_amd64__binary/) [![Build Status](http://build.ros2.org/job/Fbin_uF64__system_modes__ubuntu_focal_amd64__binary/badge/icon)](http://build.ros2.org/job/Fbin_uF64__system_modes__ubuntu_focal_amd64__binary/) | [![](https://img.shields.io/github/issues/micro-ROS/system_modes.svg)](https://github.com/micro-ROS/system_modes/issues) |
| rclc                    | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/ros2/rclc/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-dashing-brightgreen)](https://github.com/ros2/rclc/releases) [![](https://img.shields.io/badge/ROS-eloquent-brightgreen)](https://github.com/ros2/rclc/releases)[![](https://img.shields.io/badge/ROS-foxy-brightgreen)](https://github.com/ros2/rclc/releases) | [![Build Status](http://build.ros2.org/job/Dbin_uB64__rclc__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros2.org/job/Dbin_uB64__rclc__ubuntu_bionic_amd64__binary/) [![Build Status](http://build.ros2.org/job/Ebin_uB64__rclc__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros2.org/job/Ebin_uB64__rclc__ubuntu_bionic_amd64__binary/) [![Build Status](http://build.ros2.org/job/Fbin_uF64__rclc__ubuntu_focal_amd64__binary/badge/icon)](http://build.ros2.org/job/Fbin_uF64__rclc__ubuntu_focal_amd64__binary/) | [![](https://img.shields.io/github/issues/ros2/rclc.svg)](https://github.com/ros2/rclc/issues) |
