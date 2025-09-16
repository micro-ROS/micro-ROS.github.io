---
title: embeddedRTPS the new experimental middleware for micro-ROS
author: Maria Merlan
---
Alexandru Kampmann currently works at the Software for Embedded Systems Research Group, RWTH Aachen University, and he is the main contributor of [embeddedRTPS](https://github.com/embedded-software-laboratory/embeddedRTPS)

Middleware implementations such as embeddedRTPS, bring the possibility of using RTPS communication layers on mid to high range MCU with networking capabilities.

Thanks to the layer to layer compatibility of micro-ROS with the ROS 2 architecture, testing those new middlewares is just a matter of writing a new RMW implementation. As well as making some porting adjustments due to the embedded nature of micro-ROS.


From the eProsima’s micro-ROS team we want to provide the same features to the official ROS 2 embedded solution.

So, we have created a basic [rmw_embeddedrtps for micro-ROS](https://github.com/micro-ROS/rmw_embeddedrtps), that, although it is not complete, allows the basic functionality of publishing, subscribing and using ROS 2 services. All of this using the well-known micro-ROS C99 API: rclc.

**micro-ROS can be compiled either with Micro XRCE-DDS for low-mid MCUs or with embeddedRTPS for larger devices that are used in critical applications. So, the developer can easily choose the preferred middleware for any application.**

Watch this [video](https://www.youtube.com/watch?v=AHs_Ysi6IGw) to get the full detail.
Get more information [here](https://discourse.ros.org/t/embeddedrtps-the-new-experimental-middleware-for-micro-ros/22741?u=mamerlan).
