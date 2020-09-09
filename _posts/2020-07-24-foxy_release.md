---
title: micro-ROS Foxy Release
author: francesca-finocchiaro
---

We are glad to announce the release of micro-ROS Foxy and its compatibility with ROS 2 Foxy and Fast DDS 2.x.

micro-ROS Foxy supports new hardware platforms in addition to the reference Olimex STM32-E407 evaluation board, namely the Crazyflie 2.1 Drone and ST Discovery IoT kit (B-L475E-IOT01A),
and new Real-Time Operating Systems (RTOSes) in addition to the already supported NuttX, specifically FreeRTOS and Zephyr v2.3.0.

It uses the eProsima middleware Micro XRCE-DDS 1.3.0, fully compatible with the latest version of eProsima Fast DDS 2.0 for ROS 2 Foxy.

micro-ROS Foxy also features the implementation of the RCLC, an additional API that complements the ROS Client Support Library (RCL).
The RCLC library is written in the C language and implements functionalities of the standard ROS 2 RCLCPP layer, adapting them to the capabilities and needs
of the low-resource devices targeted by micro-ROS. Among them, the LET Executor and the Lifecycle.
The RCLC Executor provides a C API to manage the execution of subscription and timer callbacks, similar to the RCLCPP Executor.
It is optimized for resource-constrained devices and provides additional features that allow the manual implementation
of deterministic schedules with bounded end-to-end latencies.
The RCLC Lifecycle package provides convenience functions in C to bundle an RCL node with the ROS 2 Node Lifecycle state machine, similar to the
RCLCPP Lifecycle Node.

Finally, new demos and examples, combining the different hardware platforms and RTOSes supported and written in the new RCLC API,
have been added to the already very complete pool of out-of-the-box and easy to use micro-ROS examples.

Additionally, bug fixes and improvements have been implemented in the RMW and Micro XRCE-DDS Client library.

For a complete list of improvements and bug fixes please read the [release notes](https://github.com/micro-ROS/micro_ros_setup/releases).