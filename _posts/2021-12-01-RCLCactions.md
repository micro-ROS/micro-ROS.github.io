---
title: micro-ROS fully supports ROS 2 features
author: Maria Merlan
---

micro-ROS already supported the ROS 2 communication paradigms (pub/sub, services, parameters, lifecycle….)
From now on, micro-ROS also supports actions. This means that the RCLC now is fully completed and enables all ROS 2 features.

Therefore, the full integration with all ROS 2-based interfaces can be achieved from the plain C micro-ROS API. Also, it provides specific features such as a real time executor or static memory handling. 

The [RCLC Actions Implementation](https://github.com/ros2/rclc/pull/170) in micro-ROS supports most of the well-known features of actions like goal requests, goal cancels, feedback and so on. All of them use the dynamic-memory-free micro-ROS usual approach.

They will be included in [RCLC Rolling release v3.0.5](https://github.com/ros2/rclc/releases/tag/3.0.5)
