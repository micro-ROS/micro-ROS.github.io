---
title: micro-ROS is now on the FreeRTOS blog!
author: francesca-finocchiaro
---


[FreeRTOS](https://www.freertos.org/index.html) is a market-leading and widely used Real Time Operating System (RTOS) for microcontrollers and embedded systems, built with an emphasis on reliability and ease of use. It was one of the first RTOSes to be integrated into micro-ROS' modular software stack through the [build system](https://github.com/micro-ROS/micro_ros_setup), which enables reuse of virtually all the tools and features provided by the FreeRTOS community and partners. In a recent [post](https://www.freertos.org/2020/09/micro-ros-on-freertos.html) published on the [FreeRTOS blog](https://www.freertos.org/blog.html), we offer a technical and detailed recount on why FreeRTOS makes a lightweight and ideal RTOS over which running micro-ROS.

<img alt="FreeRTOS logo" src="/img/posts/logo-freertos.jpg" width="80%"/>


Two successful cases of integration of FreeRTOS with hardware relevant for micro-ROS’ typical target applications are the [Crazyflie 2.1 drone](https://www.bitcraze.io/products/crazyflie-2-1/) and the [ESP32 MCU](https://www.espressif.com/en/products/socs/esp32). The Crazyflie software makes profitable use of several FreeRTOS’ tools and functions, and a demo example of a micro-ROS application working with FreeRTOS on the Crazyflie can be found [here](https://www.youtube.com/watch?v=UDnSpWhkfZQ&t=14s). As for the second hardware, natively integrated with FreeRTOS and offering a ready-to-use Wi-Fi antenna and Bluetooth function, a very recent [port of micro-ROS](https://discourse.ros.org/t/micro-ros-porting-to-esp32/16101) has been carried out and practical demos are in line.

Good news is that, as the users base of both micro-ROS and FreeRTOS is rapidly expanding and compelling use-cases pop up, further integration of micro-ROS with the libraries offered by FreeRTOS and FreeRTOS+ is foreseen in the very near future.

Stay tuned!