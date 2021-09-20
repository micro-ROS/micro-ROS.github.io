---
title: Handling memory messages Tutorial for micro-ROS
author: Maria Merlan
---
Having total awareness of what ROS 2 messages and types are being used for, is essential in order to handle memory correctly in an embedded C99 environment. 

The following Tutorial “Handling messages memory in micro-ROS '' at [icro-ROS web page](https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/) explains in detail how to handle messages and types memory in micro-ROS. 

The two approaches of micro-ROS memory handling are present in this tutorial.

micro-ROS Foxy: Memory handling using traditional allocation approach.

micro-ROS Galactic: Memory handling using type support introspection.

Type Support introspection in C in micro-ROS Galactic distribution enables the new type handling API. This package is able to auto-assign memory to a certain message structure using default dynamic memory allocators. 

Thanks to the inclusion of this feature, an automated memory handling for micro-ROS types is available, enhancing the usability of the micro-ROS API.

The tools related to this feature are available in the package [micro_ros_utilities](https://github.com/micro-ROS/micro_ros_utilities) and the documentation is available [here](https://micro.ros.org/docs/api/utils/).
Check this [example](https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_types_handling/micro-ros_types_handling.ino) for more information about how to avoid the message memory initialization problems in micro-ROS.

