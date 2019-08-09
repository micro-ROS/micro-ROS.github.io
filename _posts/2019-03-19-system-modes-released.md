---
title:  "ROS 2 System Modes package released"
author: anordman
---

We just released the **[ROS 2 system modes package](https://github.com/microROS/system_modes)** that implements our attempt to allow behavioral system composition for ROS 2. *System modes* assume that the system is built from components with a lifecycle (see [ROS 2 managed nodes](https://github.com/ros2/demos/blob/master/lifecycle/README.rst)). It adds a notion of **(sub-)systems**, hierarchically grouping these nodes, as well as a notion of **modes** that determine the configuration of these nodes and (sub-)systems in terms of their parameter values.

Check the [system_modes example package](https://github.com/micro-ROS/system_modes/tree/master/system_modes_examples) for a small step-by-step example. For more information, check the [ROS wiki](http://wiki.ros.org/system_modes) and the [system_modes package on github](https://github.com/micro-ROS/system_modes).
