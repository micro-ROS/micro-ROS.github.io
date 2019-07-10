---
title: Client Library
permalink: /docs/concepts/client_library/
---

## Overview

The client library provides the micro-ROS API for the user code, i.e. for application-level micro-ROS nodes. The overall goal is to provide all relevant, major ROS 2 concepts in suitable implementation for microcontrollers. Where possible, API compatibility with ROS 2 shall be achieved for ease of portability.

In this undertaking, to minimize the long-term maintenance cost, we strive to use existing data structures and algorithms from the ROS 2 stack or to bring necessary changes in the mainline stack. In detail, this raises a lot of question regarding the applicability of existing ROS 2 layers on microcontrollers in terms of runtime efficiency, portability to different RTOS, dynamic memory management, and many more. These are explored and analyzed in a dedicated [decision paper](decision_paper/).

## Advanced Concepts

Advanced concepts developed in the context of the client library are documented separately. These are:

* [Real-Time Executor](/docs/concepts/real-time_executor/)
* [System Modes](/docs/concepts/system_modes/)
* [Embedded Transform (TF)](/docs/concepts/embedded_tf/)
