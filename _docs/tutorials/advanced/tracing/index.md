---
title: Getting started with ROS 2 tracing
author: christophebedard
permalink: /docs/tutorials/advanced/tracing/
redirect_from: /tracing/
---

1. [Introduction](#introduction)
2. [Setup](#setup)
3. [Simple tracing example](#simple-tracing-example)
4. [Callback duration analysis](#callback-duration-analysis)
5. [Upcoming work](#upcoming-work)

## Introduction

Robotic systems can be hard to analyze and debug, and one big reason is that internal processing is always changing in response to sensory input. Therefore, the ability to continuously monitor and record data about the robotic software is important, to make sure it behaves deterministically, stays within resource limits, and also for later analysis.

On modern systems, the operating system and other running software has a big influence on the exact execution of the software. Therefore, we also need information about these aspects.

Tracing is a well-established method that allows to record run-time data, which is already well integrated with operating systems. For example, we can trace when a process is being scheduled, or when I/O occurs. Current tracing systems have minimal overhead and are very configurable to reduce overhead (and data size) even further.

This post aims to introduce our ongoing effort to instrument ROS 2 and provide trace analysis tools. I'll show how we can use the instrumentation and the current analysis tools to plot callback durations, like the plot shown below.

<center>
<img src="/img/tutorials/tracing_result_plot.png" />
</center>

## Setup

We'll assume you're using Ubuntu 18.04 bionic.

First, let's [install LTTng](https://lttng.org/docs/#doc-ubuntu-ppa).

```bash
$ sudo apt-add-repository ppa:lttng/stable-2.10
$ sudo apt-get update
$ sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
```

We'll also need these Python packages to read traces and setup a tracing session through ROS.

```bash
$ sudo apt-get install python3-babeltrace python3-lttng
```

If the ROS 2 development tools and dependencies are not installed on your machine, install them by following the *System setup* section [here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/#system-setup).

Now we'll download all the necessary packages. First, create your workspace.

```bash
$ mkdir -p ~/ros2_ws/src
$ cd ros2_ws/
```

Then clone everything using the `master` `ros2.repos` file, which includes the core ROS 2 packages with instrumented `rcl` and `rclcpp`. We will also need the trace analysis tools.

#### Clone source on ROS 2 eloquent
```bash
$ wget https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing/raw/master/tracing.repos
$ vcs import src < tracing.repos
```
#### Clone source on ROS 2 dashing
You can also use tracing on dashing already, but will have to build from scratch
```bash
$ wget https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing/raw/master/all.repos
$ vcs import src < all.repos
```

#### Let's build everything and source!

```bash
$ colcon build --symlink-install
$ source install/local_setup.bash
```

## Simple tracing example

Let's try tracing with a simple ping-pong example.

The `tracetools_test` package contains two nodes we can use. The first node, `test_ping`, publishes messages on the `ping` topic and waits for a message on the `pong` topic before shutting down. The second node, `test_pong`, waits for a message on the `ping` topic, then sends a message on the `pong` topic and shuts down.

To trace these nodes, we can use the `example.launch.py` launch file in the `tracetools_launch` package.

```bash
$ ros2 launch tracetools_launch example.launch.py
```

<center>
<img src="/img/tutorials/tracing_launch.svg" style="padding: 10px;" />
</center>

As shown above, you should see a few output lines, and that's it.

By default, traces are written in the `~/.ros/tracing/` directory. You can take a look at the trace's events using `babeltrace`.

```bash
$ cd ~/.ros/tracing/
$ babeltrace my-tracing-session/
```

If you only want to see the ROS events, you can instead do:

```bash
$ babeltrace my-tracing-session/ust/
```

<center>
<img src="/img/tutorials/tracing_babeltrace.svg" style="padding: 10px;" />
</center>

The last part of the `babeltrace` output is shown above. This is a human-readable version of the raw Common Trace Format (CTF) data, which is a list of events. Each event has a timestamp, an event type, some information on the process that generated the event, and the fields corresponding to the event type. The last events of our trace are pairs of `ros2:callback_start` and `ros2:callback_end` events. Each one contains a reference to its corresponding callback.

It's now time to process the trace data! The `tracetools_analysis` package provides tools to import a trace and process it. Since reading a CTF trace is slow, it first converts it to a file which we can read much faster later on. Then we can process it to get `pandas` dataframes and use those to run analyses.

```bash
$ ros2 trace-analysis process ~/.ros/tracing/my-tracing-session/ust/
```
<center>
<img src="/img/tutorials/tracing_process.svg" style="padding: 10px;" />
</center>

The output of the `process` command is shown above. In the last dataframe, named "Callback instances," you should see three rows. The first one is the timer callback that triggered the ping-pong sequence. The second one is the ping callback, and the third one is the pong callback! Callback function symbols are shown in the previous dataframe.

This is simple, but it isn't really nice visually. We can use a Jupyter notebook to analyze the data and display the results.

## Callback duration analysis

Add the following line to the arguments of each of the two `Node` objects in your launch file, which should be under `ros2_ws/src/ros2/tracing/tracetools_launch/launch/`. It will stop the nodes from shutting down after 1 exchange.

```python
arguments=['do_more']
```

Delete the previous trace directory, and execute the launch file again. Let it run for some time (e.g. 10-20 seconds), then kill it with `Ctrl+C`.

To run an analysis that displays durations of callbacks over time, use [this Jupyter notebook](https://gitlab.com/micro-ROS/ros_tracing/tracetools_analysis/blob/master/tracetools_analysis/analysis/callback_duration.ipynb), which should be under `ros2_ws/src/tracetools_analysis/tracetools_analysis/analysis/`.

The resulting plots for the `/ping` and `/pong` subscriptions are shown below. We can see that the durations vary greatly.

<center>
<img src="/img/tutorials/tracing_analysis_plots.png" />
</center>

## Relevant links

The tracing packages can be found in the [`ros2_tracing` repo](https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing). The analysis tools can be found in the [`tracetools_analysis` repo](https://gitlab.com/micro-ROS/ros_tracing/tracetools_analysis).
