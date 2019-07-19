---
title: Embedded Transform (TF)
permalink: /docs/concepts/client_library/embedded_tf/
redirect_from: /embedded_tf/
---

Table of contents

- [Introduction and Goal](#introduction-and-goal)
- [Requirements](#requirements)
- [Design](#design)
- [Implementation of tf2_filter](#implementation-of-tf2_filter)
- [Roadmap](#roadmap)
- [Acknowledgments](#acknowledgments)


## Introduction and Goal

The TF transform graph, with its support for both a temporal history, and distributed information sources, has been a novel tool for robotics frameworks when it was released in 2008. Functionally, it is based in scene graph concepts known from computer graphics [[Foote 2013]](https://ieeexplore.ieee.org/document/6556373), but these only rarely offer distribution, and did not offer temporal histories at all (mainly, because this is not needed for frame-based rendering applications like in computer graphics). Distributed scene graphs have become more widely available also in computer graphics. In robotics, work by de Laet et al. [[De Laet et al. 2013]](https://ieeexplore.ieee.org/document/6696693) has extended transforms graphs to also contain twist (i.e., angular motion) information, and to provide more compile-time error checking. This is currently not integrated with distribution mechanisms, but could be used on a single system.
One persistent issue with transform graphs has been their resource use. ROS TF works through replicated copies of the entire transform tree at every node that uses it, and is implemented through unicast TCP connections between nodes. In systems with many dynamic parts, this has sometimes been called the ``TF firehose``, because of the large stream of incoming messages.
micro-ROS will go beyond this state of the art by running the dynamic transform tree in an embedded device, while keeping resource use to a minimum based on an analysis of the spatial and temporal details actually necessary. Further, enabling real-time queries even in the face of concurrent updates through integration will be realized through integration with the micro-ROS real-time executor. It is also planned to integrate the embedded TF will with the node lifecycle to achieve further power-savings


## Requirements

Embedded TF requirements are documented at:  [github.com/microROS/geometry2/blob/ros2/tf2_filter/docs/requirements.md](https://github.com/microROS/geometry2/blob/ros2/tf2_filter/docs/requirements.md).


## Design

The Embedded TF design is documented at:  [github.com/microROS/geometry2/blob/ros2/tf2_filter/docs/design.md](https://github.com/microROS/geometry2/blob/ros2/tf2_filter/docs/design.md).


## Implementation of tf2_filter

Implementation of tf2_filter for ROS2 and micro-ROS can be found at:  [github.com/microROS/geometry2/blob/ros2/tf2_filter/](https://github.com/microROS/geometry2/blob/ros2/tf2_filter/).


## Roadmap

**2018**
* Static filter approach in the agent allowing to specify the parts of the kinematic chain that are relevant for an application component on micro-ROS.

**2019**
* Design and implement an embedded TF implementation which is integrated with the real-time executor in a way that removes the need for costly synchronization primitives.

**2020**
* Design and implement an API extension, as well as a reference implementation for custom transform representations that increase run-time efficiency.


## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
