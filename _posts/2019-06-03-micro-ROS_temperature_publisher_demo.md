---
title: micro-ROS temperature publisher demo release
author: jfm92
---

This demo shows a use case in which one micro-ROS client publishes temperature value from a sensor using a micro-ROS bridge as a gateway to ROS 2.
To run this demo, we will use an Olimex-STM32-E407 board with an HIH6130 temperature/humidity sensor and Raspberry Pi which works as a bridge.
The Olimex board runs a micro-ROS client, in this client, we will bring-up a node and a topic, and it will publish the measured value of the sensor.
At the same time, the Olimex board is connected by serial to a Raspberry Pi, which is running a micro-ROS agent. This micro-ROS agent will act as a gateway to the ROS 2 world, publishing this topic and making it visible from any device which runs ROS 2 in the same network.

The next video shows this demo running.

<video muted width="640" height="480" align="middle" controls="controls">
    <source src="/download/Dashing_post_micro-ROS_temp_publisher.mp4" type="video/mp4">
</video>
**(This video doesn't work on Chrome)**