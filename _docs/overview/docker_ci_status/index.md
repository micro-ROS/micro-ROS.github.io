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

## Github Continuous Integration Test Status

<!--Add future CI test-->

| Repository | Description | Status
-|-|-:
|[micro-ROS-setup](https://github.com/micro-ROS/micro_ros_setup)| Micro-ROS tool to build and flash Micro-ROS to every supported platform.|[![GitHub Actions status](https://github.com/micro-ROS/micro_ros_setup/workflows/CI/badge.svg)](https://github.com/micro-ROS/micro_ros_setup/actions)
|[micro-ROS.github.io](https://github.com/micro-ROS/micro-ROS.github.io)| Micro-ROS official webpage repository.|[![GitHub Actions status](https://github.com/micro-ROS/micro-ROS.github.io/workflows/CI/badge.svg)](https://github.com/micro-ROS/micro-ROS.github.io/actions)
