---
title: Overview
permalink: /docs/tutorials/programming_rcl_rclc/overview/
redirect_from:
  - /docs/tutorials/programming_rcl_rclc/
---

In this section, you'll learn the basics of the micro-ROS C API: **rclc**.

The major concepts (publishers, subscriptions, services, timers, ...) are identical with ROS 2. They even rely on the *same* implementation, as the micro-ROS C API is based on the ROS 2 client support library (rcl), enriched with a set of convenience functions by the package [rclc](https://github.com/ros2/rclc/). That is, rclc does not add a new layer of types on top of rcl (like rclcpp and rclpy do) but only provides functions that ease the programming with the rcl types. New types are introduced only for concepts that are missing in rcl, such as the concept of an executor.

* [**Nodes**](../node/)
* [**Publishers and subscribers**](../pub_sub/)
* [**Services**](../service/)
* [**Parameters**](../parameters/)
* [**Executor and timers**](../executor/)
* [**Quality of service**](../qos/)
* [**micro-ROS utilities**](../micro-ROS/)
