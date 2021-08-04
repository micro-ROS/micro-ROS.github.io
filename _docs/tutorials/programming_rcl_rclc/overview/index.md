---
title: Overview
permalink: /docs/tutorials/programming_rcl_rclc/overview/
redirect_from:
  - /docs/tutorials/programming_rcl_rclc/
---

<img src="https://img.shields.io/badge/Written_for-Foxy-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Galactic-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Rolling-green" style="display:inline"/>

In this tutorials, you'll learn the basics of the micro-ROS C API. The major concepts (publishers, subscriptions, services,timers, ...) are identical with ROS 2. They even rely on the *same* implementation, as the micro-ROS C API is based on the ROS 2 client support library (rcl), enriched with a set of convenience functions by the package [rclc](https://github.com/ros2/rclc/). That is, rclc does not add a new layer of types on top of rcl (like rclcpp and rclpy do) but only provides functions that ease the programming with the rcl types. New types are introduced only for concepts that are missing in rcl, such as the concept of an executor.

* [**Node**](../node/)
* [**Publishers and Subscriptions**](../pub_sub/)
* [**Services**](../service/)
* [**Parameters**](../parameters/)
* [**QoS**](../qos/)
* [**micro-ROS Utils**](../micro-ROS/)
