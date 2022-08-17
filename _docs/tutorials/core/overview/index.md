---
title: Overview
permalink: /docs/tutorials/core/overview/
redirect_from:
  - /docs/tutorials/core/
  - /docs/tutorials/
---

This chapter provides a number of tutorials to learn micro-ROS and relevant tools for the different RTOSes supported by micro-ROS. If you are new to micro-ROS, we strongly suggest that you take the tutorials in the following order:

* [**First micro-ROS application on Linux**](../first_application_linux/)
    
  This tutorial teaches you how to install the micro-ROS framework and tools. Then it will guide you to develop your own first micro-ROS application under Linux. (If you already know ROS 2, you will see that the tools are well integrated with standard ROS 2.)
    
* [**First micro-ROS application on an RTOS**](../first_application_rtos/)

  In this tutorial, you will learn how to build the application from the previous tutorial for a Real-Time Operating System (RTOS). You will see how to flash a microcontroller board with the application and how to communicate with it from a microprocessor running ROS 2 on Linux. (The tutorial covers all three RTOS supported by micro-ROS, namely NuttX, FreeRTOS, and Zephyr. The choice is up to you!)

Then, at this point, you may head over to the next section [**Programming with rcl and rclc**](../../programming_rcl_rclc/), where you will learn the concepts of the micro-ROS C API in this tutorial in depth. If you are already familiar with the ROS 2 C++ API or even the underlying ROS Client Support Library (rcl), you'll learn this very quickly.

In case you are using the corresponding RTOS or hardware, the following basic tutorials may be interesting before switching to the [**Programming with rcl and rclc**](../../programming_rcl_rclc/) section:

* [**Zephyr Emulator**](../zephyr_emulator/)

  In this tutorial, you'll learn the use of micro-ROS with Zephyr emulator by testing a Ping Pong application.
  
* [**Teensy with Arduino**](../teensy_with_arduino/)
     
  In this tutorial you will learn how to connect Teensy with micro-ROS and ROS 2. You will also learn how to install micro-ROS agent in linux systems to communicate with Teensy based arduino board using Arduino IDE. This tutorial will also cover a simple publisher topic published from teensy and subscribed using ROS2 interface. 

 In the [**Advanced Tutorials**](../../advanced/overview/) section, you'll find more advanced tutorials to strenghten your micro-ROS knowledge.
