---
title: Lifecycle and System Modes
permalink: /docs/system_modes/
redirect_from: /system_modes/
---


Table of contents
- [Introduction and Goal](#introduction-and-goal)
- [Requirements](#requirements)
- [Background: ROS 2 Lifecycle](#background-ros-2-lifecycle)
- [Main Features](#main-features)
  - [Extended Lifecycle](#extended-lifecycle)
  - [System Hierarchy and Modes](#system-hierarchy-and-modes)
  - [Mode manager](#mode-manager)
- [Roadmap](#roadmap)
- [Acknowledgments](#acknowledgments)


## Introduction and Goal

Modern robotic software architectures often follow a layered approach. The layer with the core algorithms for SLAM, vision-based object recognition, motion planning, etc. is often referred to as *skill layer* or *functional layer*. To perform a complex task, these skills are orchestrated by one or more upper layers named *executive layer and planning layer*. Other common names are *task and mission layer* or *deliberation layer(s)*. In the following, we used the latter term.

We observed three different but closely interwoven aspects to be handled on the deliberation layer:

1.  **Task Handling**: Orchestration of the actual task, the *straight-forward*, *error-free* flow.
2.  **Contingency Handling**: Handling of task-specific contingencies, e.g., expectable retries and failure attempts, obstacles, low battery.
3.  **System Error Handling**: Handling of exceptions, e.g., sensor/actuator failures.

The mechanisms being used to orchestrate the skills are service and action calls, re-parameterizations, set values, activating/deactivating of components, etc. We distinguish between *function-oriented calls* to a running skill component (set values, action queries, etc.) and *system-oriented calls* to individual or multiple components (switching between component modes, restart, shutdown, etc.).

![Interaction between skill and deliberation layer](interactions_between_skill_and_deliberation_layer.png)

Analogously, we distinguish between *function-oriented notifications* from the skill layer in form a feedback on long-running service calls, messages on relevant events in the environment, etc. and *system-oriented notifications* about component failures, hardware errors, etc.

Our observation is that interweaving of task handling, contingency handling, and system error handling generally leads to a high complexity of the control flow on the deliberation layer. Yet, we hypothesize that this complexity can be reduced by introducing appropriate abstractions for system-oriented calls and notifications.

Therefore, our **goal** within this work is to provide suitable abstractions and framework functions for (1.) system runtime configuration and (2.) system error and contingency diagnosis, to reduce the effort for the application developer of designing and implementing the task, contingency and error handling.

This goal is illustrated in the following high-level architecture:

![High-level Architecture](goal.png)

The envisioned key elements to achieve this goal are:

1.  Extensible concept to specify the runtime states of components, i.e ROS 2 nodes.
2.  Modeling approach for specifying system modes based on these component states.
3.  Diagnosis module for deriving relevant information from the operating systems, the hardware and the functional components.
4.  Mode manager module for system runtime configuration.

## Requirements

The list of requirements is maintained in the doc folder of the micro-ROS system modes repository, at:  https://github.com/micro-ROS/system_modes/blob/master/system_modes/doc/requirements.md


## Background: ROS 2 Lifecycle

Our approach is based on the ROS 2 Lifecycle. The primary goal of the ROS 2 lifecycle is to allows greater control over the state of a ROS system. It allows consistent initialization, restart and/or replacing of system parts during runtime. It provides a default lifecycle for managed ROS 2 nodes and a matching set of tools for managing lifecycle nodes.

The description of the concept can be found at:   [http://design.ros2.org/articles/node_lifecycle.html](http://design.ros2.org/articles/node_lifecycle.html)  
The implementation of the Lifecycle Node is described at:  
[https://index.ros.org/doc/ros2/Managed-Nodes/](https://index.ros.org/doc/ros2/Managed-Nodes/).

## Main Features

### Extended Lifecycle

In micro-ROS, we extend the ROS 2 lifecycle by allowing to specify modes, i.e. substates, specializing the *active* *state based on the standard ROS 2 parameters mechanism. We implemented this concept based on rcl and rclcpp for ROS 2 and micro-ROS.

Documentation and code can be found at:  
[github.com:system_modes/README.md#lifecycle](https://github.com/micro-ROS/system_modes/blob/master/system_modes/README.md#lifecycle)


### System Hierarchy and Modes

We provide a modeling concept for specifying the hierarchical composition of systems recursively from nodes and for specifying the states and modes of systems and subsystems with the extended lifecycle, analogously to nodes. This system modes and hierarchy (SMH) model also includes an application-specific the mapping of the states and modes along the system hierarchy down to nodes.

The description of this model can be found at:  
[github.com:system_modes/README.md#system-modes](https://github.com/micro-ROS/system_modes/blob/master/system_modes/README.md#system-modes)  
A simple example is provided at:  
[github.com:system_modes_examples/README.md#example-mode-file](https://github.com/micro-ROS/system_modes/blob/master/system_modes_examples/README.md#example-mode-file)


### Mode manager

The mode manager allows for runtime system adaptation based on such a system hierarchy and modes model. It parses the model and provides all services and topics to request state and mode changes and to monitor these changes.

The documentation and code can be found at:  
[github.com:system_modes/README.md#mode-manager](https://github.com/micro-ROS/system_modes/blob/master/system_modes/README.md#mode-manager)  
A simple example is provided at:  
[github.com:system_modes_examples/README.md#setup](https://github.com/micro-ROS/system_modes/blob/master/system_modes_examples/README.md#setup)


## Roadmap

**2018**
*   Extended lifecycle concept and implementation for ROS 2 and micro-ROS.
*   Modeling concept to specify system hierarchy as well as system modes of systems, subsystems, and their mapping along the system hierarchy down to nodes.
*   Mode inference and mode manager in C++ for ROS 2.

**2019**
*   Specific implementation of mode manager for micro-ROS as may be necessary.
*   Diagnostics framework for micro-ROS, interoperating with ROS 2 diagnostics.
*   MCU-specific diagnostics functions for resource usage on RTOS layer, latencies, statistics from middleware, etc.
*   Integration of mode manager with real-time executor and/or roslaunch.

**2020**
*   Lightweight concept for specifying error propagations between nodes and subsystems.

_Note: The extension of the ACTIVE state by modes (substates) was originally planned for 2020 but brought forward in 2018._


## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
