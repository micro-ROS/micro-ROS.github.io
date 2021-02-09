---
title: Introduction to Client Library
permalink: /docs/concepts/client_library/introduction/
redirect_from:
  - /docs/concepts/
  - /docs/concepts/client_library/
---

The Client Library provides the micro-ROS API for the user code, i.e., for application-level micro-ROS nodes. The goal is to provide all relevant, major ROS 2 concepts in an optimized implementation for microcontrollers. At the same time, we strive to make the API as compatible as possible to standard ROS 2, to facilitate porting of code.

To minimize the long-term maintenance cost, we use existing data structures and algorithms from the ROS 2 stack and bring necessary changes in the mainline stack as far as possible. That's why the micro-ROS client library is built up from standard [ROS 2 Client Support Library (rcl)](https://github.com/ros2/rcl/) and a new [ROS 2 Client Library package (rclc)](https://github.com/ros2/rclc/). Together, as depicted below, rcl + rclc form a feature-complete client library in the C programming language.

<img src="/img/micro-ROS_architecture.png" style="display:block; width:50%; float:right;"/>

Important features and properties:

* Use of rcl data structures where possible to avoid runtime overhead by wrappers.
* Convenience functions for common tasks (e.g., creation of a publisher, finalization of a subscription) provided by rclc.
* Dedicated Executor for fine-grained control over triggering and processing order of callbacks.
* Specialized implementations for graphs, lifecycle nodes, diagnostics, etc.

Check out the subpages (see left) for more information.

<br style="clear:both;" />

For the interested reader: The rationales for the decision to use a combination of rcl + rclc are explained in our [decision paper (PDF)](/download/client_library_decision_paper_2019.pdf) from 2019.
