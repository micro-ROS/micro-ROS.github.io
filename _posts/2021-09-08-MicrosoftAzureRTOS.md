---
title: Microsoft Azure RTOS integrates micro-ROS
author: Maria Merlan
---
The micro-ROS framework has been fully integrated in Azure RTOS embedded development suite.
This port [(github repo)](https://github.com/micro-ROS/micro_ros_azure_rtos_app) was done using a STMicroelectronics B-L475E-IOT01A. 

This achievement brings the following benefits: 
- To seamlessly integrate ROS 2 compatible nodes in Azure RTOS supported hardware.
- To have a full featured API with all ROS 2 concepts (such as publishers, subscribers, services or executors) in the far edge of the robotic application and integrated in the Azure RTOS build system.  
- To integrate micro-ROS and ROS 2 applications taking advantage of the complete set of functionality of ThreadX kernel and the whole Azure RTOS ecosystem.
- To implement custom communication mechanisms between embedded platforms and ROS 2 using Azure RTOS APIs.

Check this [video](https://www.youtube.com/watch?v=RsnHEaD8b9E) on “Getting started with micro-ROS and Azure RTOS using STMicroelectronics B-L475E-IOT01A” to get the insights and reproduce the integration of micro-ROS in any Azure RTOS supported HW. 

[Azure RTOS](https://github.com/azure-rtos), developed by Microsoft in C, is an embedded development suite including an operating system that provides reliable, ultra-fast performance for resource-constrained devices. Azure RTOS supports the most popular 32-bit microcontrollers and embedded development tools.
