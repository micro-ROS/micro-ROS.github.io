---
title: micro-ROS Multithreading support
author: Maria Merlan
---
[micro-ROS](https://micro.ros.org/) Foxy, Galactic and Rolling provides a new enhancement, the multithreading feature. 

Multithreading allows the micro-ROS default middleware (Micro XRCE-DDS) to be thread safe. This way, micro-ROS nodes can run ROS 2 entities such as publishers, subscribers or services in different RTOS execution threads.
As a result, micro-ROS will run in a multi threaded approach under specific circumstances, improving the performance, the usability and resource optimization.

The multithreading mechanism together with shared memory [transport](https://discourse.ros.org/t/shared-memory-implementation-available-now-in-micro-ros/20422?u=mamerlan) contribute to provide a highly flexible environment for deploying micro-ROS nodes with isolated functionality in different RTOS tasks or threads. As always, in a build-time configured way with no usage of dynamic memory.

→ To activate this new feature, just enable the Multithreading [flag](https://github.com/eProsima/Micro-XRCE-DDS-Client/blob/3e2a144b44add2a789ecdfd116686c981738839e/CMakeLists.txt#L65) in your middleware configuration using your colcon.meta file.

These features are supported in Micro XRCE-DDS, the micro-ROS middleware implementation.

See the associated PRs in the Micro XRCE-DDS library ([General](https://github.com/eProsima/Micro-XRCE-DDS/pull/85), [Client](https://github.com/eProsima/Micro-XRCE-DDS-Client/pull/224), [Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent/pull/239)) and in micro-ROS’ Client RMW and Agent.
