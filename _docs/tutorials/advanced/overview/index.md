---
title: Overview
permalink: /docs/tutorials/advanced/overview/
redirect_from:
  - /docs/tutorials/advanced/
---

This chapter provides a number of advanced tutorials for those users who already have some previous micro-ROS knowledge. They are useful to interact with micro-ROS at a deeper level compared with the [**First Step Tutorials**](../../core/overview). There is no specific order recommended to take these tutorials, as each addresses a different aspect of the micro-ROS stack and toolchain.

* [**Optimizing the Middleware Configuration**](../microxrcedds_rmw_configuration/)

  In this tutorial, we'll guide you through the configuration of the middleware between a microcontroller and the micro-ROS agent running on some Linux-based microprocessor, to optimize it for your specific use-case and application.

* [**How to include a custom ROS message in micro-ROS**](../create_new_type/)

  This tutorial explains how to create or include a custom ROS message type in a micro-ROS application - and in particular how to bring it into the [build system](https://github.com/micro-ROS/micro_ros_setup).

* [**How to use custom QoS in micro-ROS**](../create_dds_entities_by_ref/)

  This tutorial explains the procedure for creating micro-ROS entities using fully configurable QoS settings by using the ROS 2 (DDS) entities creation mode *by references* as allowed by the micro-ROS default middleware (Micro XRCE-DDS Client).

* [**Creating custom micro-ROS transports**](../create_custom_transports/)

  This tutorial aims at providing step-by-step guidance for those users interested in creating micro-ROS custom transports, instead of using the ones provided by default in the micro-ROS tools set.

* [**Creating custom static micro-ROS library**](../create_custom_static_library/)

  This tutorial aims at providing step-by-step guidance for those users interested in compiling micro-ROS as a standalone library in order to integrate it in custom development tools.

* [**Benchmarking with the Shadow-Builder**](../benchmarking/)

  This tutorial aims at describing a specific benchmarking tooling called the *Shadow Builder*. More specifically, it explains how to create a plugin from A to Z and how to instrument the code.
