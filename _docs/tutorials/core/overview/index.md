---
title: Overview
permalink: /docs/tutorials/core/overview/
redirect_from:
  - /docs/tutorials/core/
  - /docs/tutorials/
---

This chapter provides you a number of tutorials to learn micro-ROS and relevant tools for the different RTOS supported by micro-ROS. We divided this chapter into two sections core tutorials and advanced tutorials.

If you are new to micro-ROS, we strongly suggest that you take the tutorials in the following order:

* [**First micro-ROS application on Linux**](../first_application_linux/)

  This tutorial teaches you how to install the micro-ROS framework and tools. Then it will guide you to developed your own first micro-ROS application under Linux. (If you already know ROS 2, you will see that the tools are well integrated with standard ROS 2.)

* [**First micro-ROS application on an RTOS**](../first_application_RTOS/)

  In this tutorial, you will learn how to build the application from the previous tutorial for an Real-Time Operating System (RTOS). You will see how to flash a microcontroller board with the application and how to communicate with it from a microprocessor running ROS 2 on Linux. (The tutorial covers all three RTOS supported by micro-ROS, namely NuttX, FreeRTOS, and Zephyr. The choice is up to you!)

* [**Programming with rcl and rclc**](../programming_rcl_rclc/)

  You'll learn the concepts of the micro-ROS C API in this tutorial in depth. If you are already familiar with the ROS 2 C++ API or even the underlying ROS Client Support Library (rcl), you'll learn this very quickly.

* [**Optimizing the Middleware Configuration**](../microxrcedds_rmw_configuration/)

  In this tutorial, we'll guide you through the configuration of the middleware between a microcontroller and the micro-ROS agent running on some Linux-based microprocessor, to optimize it for your specific use-case an application.
