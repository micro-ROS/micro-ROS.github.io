---
title: Comparison to other approaches to put ROS onto small embedded devices
permalink: /docs/overview/comparison/
---

This page compares micro-ROS to a few alternative approaches.

## micro-ROS compared to ROSSerial

In ROS1, small embedded devices are brought into the system using `rosserial`.

Differences are:

|  | rosserial | micro-ROS |
|-------|-----------|-----------|
| OS | bare-metal | POSIX (currently NuttX) |
| Communications architecture | Bridged | Bridged |
| Message format | ROS1 | CDR (from DDS) |
| Communication links | UART | UART, SPI, IP (UDP), 6LowPAN, ... |
| Communication protocol | Custom | XRCE-DDS (or any rmw implementation) |
| Code Base | Independent implementation | Standard ROS 2 stack up to RCL (RCLCPP coming) |
| Node API | Custom rosserial API | RCL (soon RCLCPP) |
| Callback execution | Sequential, in order of messages | Choice of ROS 2 executors or MCU optimized executors |
| Timers | Not included | Normal ROS 2 timers |
| Time sync to host | Custom | NTP/PTP |
| Lifecycle | Not supported | Partial, full coming |
