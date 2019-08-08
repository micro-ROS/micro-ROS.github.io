---
title: Real-Time Executor
redirect_from: /real-time_executor/
permalink: /docs/real-time_executor/
---


TODO:
- documentation in github readme.md im packag
dann kann man es auch unter ros.index wiederfinden
mit beispielen, wie man es verwendet.

- related work Orocos


## Table of contents

*   [Introduction](#introduction)
*   [ROS 2 Executor Concept](#ros-2-executor-concept)
*   [RCL Executor](#rcl-executor)
    * [Client library layers](#api-layers)
    * [let executor](#LET-executor)
    * [API](#API)
*   [Background](#background)
    *   [rclcpp executor](#rclcpp-executor)
        *   [Description](#description)
        *   [Architecture](#architecture)
        *   [Analysis](#analysis)
    * [EDF Scheduler for ROS2](#EDF-scheduler-ROS2)
        * analyse rclcpp
        * EDF approach
    *   [Callback-group-level Executor](#callback-group-level-executor)
        *   [API Changes](#api-changes)
        *   [Meta-Executor Concept](#meta-executor-concept)
        *   [Test Bench](#test-bench)

*   [Roadmap](#roadmap)
*   [Related Work](#related-work)
*   [References](#references)
*   [Acknowledgments](#acknowledgments)


## Introduction

Predictable execution under given real-time constraints is a crucial requirement for many robotic applications, but is also a complex and time-consuming activity. While the service-based paradigm of ROS allows a fast integration of many different functionalities, it does not provide sufficient control over the execution management. For example,  there are no mechanisms to enforce a certain execution order of callbacks within a node, left alone to configure the  execution order of callbacks of different nodes in a ROS 2 system. This is essential for a high quality of control applications, which consist of cause-effect-chains like sensor acquisition, evaluation and actuation control, and operating on old data due to scheduling is not desired. Further more, when input data is collected in field tests, saved with ROS-bags and re-played, often results are different due to non-determinism of process scheduling.

Of course, it is possible to manually setup the order of subscribing and publishing topics in the callbacks or by tweaking the priorities of the corresponding Linux processes. However, this approach is error-prune, difficult to extend and requires an in-depth knowledge of the deplyed ROS 2 packages in the system.

Therefore the goal of the Real-Time Executor is to support roboticists with practical and easy-to-use real-time mechanisms which provide solutions for:
- Deterministic execution
- Real time guarantees
- Integration of real-time and non real-time functionalities on one platform
- Specific support for RTOS and microcontrollers

The challenges to achieve these goals are:
- to develop an adequate and well-defined scheduling mechanisms for the ROS framework and the operating system (RTOS)
- to define an easy-to-use interface for ROS-developers
- to model requirements (like latencies, determinism in subsystems)
- mapping of ROS framework and OS scheduler (semi-automated and optimized mapping is desired as well as generic, well-understood framework mechanisms)

Our approach is to provide a real-time Executor on two layers (as described HERE). One based on rcl written in C programming language and one based on rclcpp written in C++.

As the first step, we propose an RCL Executor, which implements a very simple execution concept, namely a static order scheduling. In this scheduling policy, all processes are executed in a pre-defined order. Secondly, we developed a Callback-group-level executor, which allows to prioritize a group of callbacks. These approaches are based on the concept of executors, which have been introduced in ROS 2.

The remaining of the chapter is structured as follows. First, the ROS 2 Executor concept is described. Then we describe the RCL Executor. In the background section we go into more detail of the ROS 2 Executor (rclcpp), point out its limitations, as published in a recent paper [1] and describe the callback-group-level Executor. Finally, we summarize related work and present the OFERA project roadmap.

## ROS 2 Executor Concept

ROS 2 allows to bundle multiple nodes in one operating system process. To coordinate the execution of the callbacks of the nodes of a process, the Executor concept was introduced in rclcpp (and also in rclpy).

The ROS 2 design defines one Executor (instance of [rclcpp::executor::Executor](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executor.hpp)) per process, which is typically created either in a custom main function or by the launch system. The Executor coordinates the execution of all callbacks issued by these nodes by checking for available work (timers, services, messages, subscriptions, etc.) from the DDS queue and dispatching it to one or more threads, implemented in [SingleThreadedExecutor](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executors/single_threaded_executor.hpp) and [MultiThreadedExecutor](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executors/multi_threaded_executor.hpp), respectively.

The dispatching mechanism resembles the ROS 1 spin thread behavior: the Executor looks up the wait queues, which notifies it of any pending callback in the DDS queue. If there are multiple pending callbacks, the ROS 2 Executor executes them in an unspecified order.

## RCL-Executor

### ROS 2 Layers
As mentioned in [Introduction to Client Library](../index.md) 

### User API
The RCL-Executor is library written in C and is based on RCL. On implementation level we use a different terminology. In the RCL-Executor library an *event* is called a *handle* and *static-order* scheduling is called *FIFO* scheduling.

The RCL-Executor provides the following user interface:
* Initialization
  * Total number of handles
  * Scheduling policy (*FIFO*, *PRIORITY*)
* Configuration
  * rcle_executor_add_subscription()
  * rcle_executor_add_subscription_prio()
  * rcle_executor_add_timer()
  * rcle_executor_add_subscription_prio()
* Running
  * rcle_executor_spin_once()
  * rcle_executor_spin()

For the static-order scheduling, the user initializes the RCL-Executor with *FIFO* as scheduling policy and the total number of handles. Then, the handles are added the the RCL-Executor. The order of these method-calls defines the static-order for the scheduling, hence the name *FIFO* scheduling. Processing the handles is started by e.g. the *spin*-function.

For priority-based scheduling, the user initializes the RCL-Executor with *PRIORITY* as scheduling policy and the total number of handles. The specific priority of each handle is specified in the respective *rcle_executor_add-\*-prio* function. Processing the handles is started by e.g. the *spin*-function.

As resources are very constrained on micro-controllers, specific attention has been paid to the memory allocation: Dynamic memory is only allocated during the initialization phase to reserve memory for all handles. Later on, no memory is allocated while scheduling the handles (which was the case in the RCLC implementation).

The RCL-Executor and examples can be found in the repository [rcl-executor](https://github.com/micro-ROS/rcl_executor).


## Static-order scheduler
RT Executors

implementation:
- as thin-layer based on RCL (written in C)

API
- create executor
- add_subscription
- add_timer
- add_function
- spin_some
- spin_period

semantics:
fixed static order scheduling. All handles (callbacks, timers, services, etc.) are executed in the order
as they were added using the add_* methods to the executor.

LET semantics (reference to some scientific paper)
- read first, process, write
- benefit: no interference of write of some callbacks to any other callbacks in this round of evaluating DDS queue ()
take section from software architecture deliverable

implementation notes:
- implemented by: 1) creating a wait_set 2) rcl_wait() 3) rcl_take (for all ready handles) 4) scheduling of all handles in the static ORDER
- during configuration: dynamic allocation for n number of handles, during run-time (e.g. spin_some) no dynamic memory allocation (except within rcl_take(), where memory might be dynamically allocated for messages with variable length)




## Background

### ROS2 rclcpp Executor

#### Architecture

The following diagram depicts the relevant classes of the ROS 2 Executor concept:

![ROS 2 Executor class diagram](executor_class_diagram.png)

Note that an Executor instance maintains weak pointers to the NodeBaseInterfaces of the nodes only. Therefore, nodes can be destroyed safely, without notifying the Executor.

Also, the Executor does not maintain an explicit callback queue, but relies on the queue mechanism of the underlying DDS implementation as illustrated in the following sequence diagram:

![Call sequence from executor to DDS](executor_to_dds_sequence_diagram.png)

#### Analysis
The Executor concept, however, does not provide means for prioritization or categorization of the incoming callback calls. Moreover, it does not leverage the real-time characteristics of the underlying operating-system scheduler to have finer control on the order of executions. The overall implication of this behavior is that time-critical callbacks could suffer possible deadline misses and a degraded performance since they are serviced later than non-critical callbacks. Additionally, due to the FIFO mechanism, it is difficult to determine usable bounds on the worst-case latency that each callback execution may incur.

OLD STUFF

The ROS 2 Executor is responsible for receiving incoming data, calling timers, services etc. The recent paper ) analyzed the Executor in detail. They found out, that these events are not processed in a pure FIFO fashion, but timers are preferred over all other events of the DDS queue. The implication is, that in a high-load situation, only pending timers will be processed while incoming DDS-events will be delayed or starved. Furthermore, the FIFO-strategy makes it difficult to determine time bounds on the total execution time, which are necessary for the verification of safety- and real-time requirements.

To solve these issues, we have implemented an RCL-Executor, which provides different scheduling strategies. Currently, two scheduling policies are implemented: static-order and priority-based scheduling.

In static-order-scheduling, all events, including timers, are processed in a pre-defined order. In case different events (timers, subscriptions, etc.) are available at the DDS queue, then they are all processed in the pre-defined static order.

In priority-based scheduling, a priority is defined for each event. If multiple events are available at the DDS queue, then only the event with the highest priority is processed.   

### EDF for ROS

 Tobias Blass PAPER

### Callback-group-level Executor
 We propose a callback-group-level Executor, which is based on the Executor concept of the ROS 2 Executor and addresses its afore-mentioned deficits. This new Executor provides fine-grained control of the mapping of callbacks to the scheduling primitives and mechanisms of the underlying RTOS, even across multiple nodes. ROS 2 allows bundling multiple nodes in one operating system process. To coordinate the execution of the callbacks of the nodes of a process, the Executor concept was introduced in rclcpp (and also in rclpy).

#### Description
In order to address the challenges mentioned above, some changes are imminent: Firstly, an API to express real-time requirements on a callback level is needed and secondly, the Executor must be redesigned to respect these real-time requirements in its scheduling decisions. As the current ROS 2 Executor works at a node-level granularity – which is a limitation given that a node may issue different callbacks needing different real-time guarantees - we decided to refine the ROS 2 Executor API for more fine-grained control over the scheduling of callbacks on the granularity of callback groups using. We leverage the callback-group concept existing in rclcpp by introducing real-time profiles such as RT-CRITICAL and BEST-EFFORT in the callback-group API (i.e. rclcpp/callback_group.hpp). Each callback needing specific real-time guarantees, when created, may therefore be associated with a dedicated callback group. With this in place, we enhanced the Executor and depending classes (e.g., for memory allocation) to operate at a finer callback-group granularity. This allows a single node to have callbacks with different real-time profiles assigned to different Executor instances - within one process.

Thus, an Executor instance can be dedicated to specific callback group(s) and the Executor’s thread(s) can be prioritized according to the real-time requirements of these groups. For example, all time-critical callbacks are handled by an "RT-CRITICAL" Executor instance running at the highest scheduler priority.

The following figure illustrates this approach with two nodes served by three Callback-group-level Executors in one process:

![Sample system with two nodes and three Callback-group-level Executors in one process](cbg-executor_sample_system.png)

The different callbacks of the Drive-Base node are distributed to different Executors (visualized by the color red, yellow and green).  For example the onCmdVel and publishWheelTicks callback are scheduled by the same Executor (yellow). Callbacks from different nodes can be serviced by the same Executor.

#### API Changes
In this section, we describe the necessary changes to the Executor API:
*   [include/rclcpp/callback\_group.hpp](https://github.com/micro-ROS/rclcpp/blob/cbg-executor-0.5.1/rclcpp/include/rclcpp/callback_group.hpp):

    * Introduced an enum to distinguish up to three real-time classes (requirements) per node (RealTimeCritical, SoftRealTime, BestEffort)
    * Changed association with Executor instance from nodes to callback groups.
*   [include/rclcpp/executor.hpp](https://github.com/micro-ROS/rclcpp/blob/cbg-executor-0.5.1/rclcpp/include/rclcpp/executor.hpp)

    * Added functions to add and remove individual callback groups in addition to whole nodes.

    * Replaced private vector of nodes with a map from callback groups to nodes.

*   [include/rclcpp/memory\_strategy.hpp](https://github.com/micro-ROS/rclcpp/blob/cbg-executor-0.5.1/rclcpp/include/rclcpp/memory_strategy.hpp)

    * Changed all functions that expect a vector of nodes to the just mentioned map.
*   [include/rclcpp/node.hpp](https://github.com/micro-ROS/rclcpp/blob/cbg-executor-0.5.1/rclcpp/include/rclcpp/node.hpp) and [include/rclcpp/node_interfaces/node_base.hpp](https://github.com/micro-ROS/rclcpp/blob/cbg-executor-0.5.1/rclcpp/include/rclcpp/node_interfaces/node_base.hpp)

    * Extended arguments of create\_callback\_group function for the real-time class.
    * Removed the get\_associated\_with\_executor\_atomic function.

All the changes can be found in the branches [cbg-executor-0.5.1](https://github.com/micro-ROS/rclcpp/tree/cbg-executor-0.5.1/rclcpp) and [cbg-executor-0.6.1](https://github.com/micro-ROS/rclcpp/tree/cbg-executor-0.6.1/rclcpp) for the corresponding version 0.5.1 and 0.6.1 of the rclcpp in the fork at [github.com/micro-ROS/rclcpp/](https://github.com/micro-ROS/rclcpp/).

#### Meta-Executor Concept

The idea of the Meta Executor is to abstract away the callback-group assignment, thread allocation and other inner workings of the Executors from the user, thereby presenting a simple API that resembles the original Executor interface. Internally, the Meta Executor maintains multiple instances of our Callback-group-level Executor (Cbg-Executor).

The Meta Executor internally binds these Executors to the underlying kernel threads, assigns them a priority, chooses the scheduling mechanism (e.g., SCHED-FIFO policy) and then dispatches them. When adding a node with its list of callback group and real-time profiles to the Meta Executor, it parses the real-time profiles and assigns the node’s callback groups to the relevant internal Executors.

![Illustration of the Meta-Executor concept](meta-executor_concept.png)

#### Test Bench

As a proof of concept, we implemented a small test bench in the present package cbg-executor_ping-pong_cpp. The test bench comprises a Ping node and a Pong node which exchange real-time and best-effort messages simultaneously with each other. Each class of messages is handled with a dedicated Executor, as illustrated in the following figure.

![Architecture for the Callback-group-level Executor test bench](cbg-executor_test-bench_architecture.png)
With the test bench, we validated the functioning of the approach - here on ROS 2 v0.5.1 with the Fast-RTPS DDS implementation - on a typical laptop.

![Results from Callback-group-level Executor test bench](cbg-executor_test-bench_results.png)

The test bench is provided in the [bg-executor_ping-pong_cpp](https://github.com/microROS/micro-ROS-demos/tree/master/Cpp/cbg-executor_ping-pong) package of the [micro-ROS-demos](https://github.com/microROS/micro-ROS-demos/) repository.



## Related Work

In this section, we provide an overview to related approaches and link to the corresponding APIs.


### Fawkes Framework

[Fawkes](http://www.fawkesrobotics.org/) is a robotic software framework, which supports synchronization points for sense-plan-act like execution. It has been developed by RWTH Aachen since 2006. Source code is available at [github.com/fawkesrobotics](https://github.com/fawkesrobotics).

#### Synchronization
Fawkes provides developers different synchronization points, which are very useful for defining an execution order of a typical sense-plan-act application. These ten synchronization points (wake-up hooks) are the following (cf. [libs/aspect/blocked_timing.h](https://github.com/fawkesrobotics/fawkes/blob/master/src/libs/aspect/blocked_timing.h)):

*   WAKEUP\_HOOK\_PRE\_LOOP
*   WAKEUP\_HOOK\_SENSOR\_ACQUIRE
*   WAKEUP\_HOOK\_SENSOR\_PREPARE
*   WAKEUP\_HOOK\_SENSOR\_PROCESS
*   WAKEUP\_HOOK\_WORLDSTATE
*   WAKEUP\_HOOK\_THINK
*   WAKEUP\_HOOK\_SKILL   
*   WAKEUP\_HOOK\_ACT     
*   WAKEUP\_HOOK\_ACT\_EXEC
*   WAKEUP\_HOOK\_POST\_LOOP  

#### Configuration at compile time
At compile time, a desired synchronization point is defined as a constructor parameter for a module. For example, assuming that `mapLaserGenThread` shall be executed in SENSOR_ACQUIRE, the constructor is implemented as:

```C++
MapLaserGenThread::MapLaserGenThread()
  :: Thread("MapLaserGenThread", Thread::OPMODE_WAITFORWAKEUP),
     BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
     TransformAspect(TransformAspect::BOTH_DEFER_PUBLISHER, "Map Laser Odometry")
```

Similarly, if `NaoQiButtonThread` shall be executed in the SENSOR_PROCESS hook, the constructor is:

```C++
NaoQiButtonThread::NaoQiButtonThread()
  :: Thread("NaoQiButtonThread", Thread::OPMODE_WAITFORWAKEUP),
     BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
```
#### Runtime execution
At runtime, the *executor* iterates through the list of synchronization points and executes all registered threads until completion. Then, the threads of the next synchronization point are called.

A module (thread) can be configured independent of these sense-plan-act synchronization points. This has the effect, that this thread is executed in parallel to this chain.

The high level overview of the Fawkes framework is shown in the next figure. At compile-time the configuration of the sense-plan act wakeup hook is done (upper part), while at run-time the scheduler iterates through this list of wakeup-hooks (lower part):

![Sequence diagram for executor concept in Fawkes](fawkes_executor_diagram.png)

Hence, at run-time, the hooks are executed as a fixed static schedule without preemption. Multiple threads registered in the same hook are executed in parallel.

Orthogonal to the sequential execution of sense-plan-act like applications, it is possible to define further constraints on the execution order by means of a `Barrier`. A barrier defines a number of threads, which need to have finished before the thread can start, which owns the barrier.

These concepts are implemented by the following main classes:

*   *Wakeup hook* by `SyncPoint` and `SyncPointManager`, which manages a list of synchronization points.
*   *Executor* by the class `FawkesMainThread`, which is the scheduler, responsible for calling the user threads.
*   `ThreadManager`, which is derived from `BlockedTimingExecutor`, provides the necessary API to add and remove threads to wakeup hooks as well as for sequential execution of the wakeup-hooks.
- `Barrier` is an object similar to `condition_variable` in C++.

####Discussion

All threads are executed with the same priority. If multiple sense-plan-act chains shall be executed with different priorities, e.g. to prefer execution of emergency-stop over normal operation, then this framework reaches its limits.

Also, different execution frequencies cannot be modeled by a single instance of this sense-plan-act chain. However, in robotics the fastest sensor will drive the chain and all other hooks are executed with the same frequency.

The option to execute threads independent of the predefined wakeup-hooks is very useful, e.g. for diagnostics. The concept of the Barrier is useful for satisfying functional dependencies which need to be considered in the execution order.


<!--
### Orocos

TODO INSERT DESCRIPTION ON PARTIAL ORDER SCHEDULING.


### CoSiMA

TODO INSERT DESCRIPTION ON MODEL-BASED APPROACH BY COSIMA (ON TOP OF OROCOS) FROM FOLLOWING PAPER:

D. L. Wigand, P. Mohammadi, E. M. Hoffman, N. G. Tsagarakis, J. J. Steil and S. Wrede, "An open-source architecture for simulation, execution and analysis of real-time robotics systems," 2018 IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR), Brisbane, QLD, 2018, pp. 93-100.
doi: 10.1109/SIMPAR.2018.8376277
URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8376277&isnumber=8376259
-->

## Roadmap

**2018**

*   In-depth analysis of the ROS 2 Executor concept.
*   Designed Callback-group-level Executor and implemented it - at first, prototypically, for ROS 2.
*   Designed the Meta Executor concept.
*   Validated correct functioning with underlying layers and middleware.

**2019**

*   Implementation of the Callback-group-level Executor and Meta Executor for urcl.
*   Research of concepts for model-based optimization of end-to-end latencies.

**2020**

*   Integration with a selected advanced scheduling and resource monitoring mechanisms such as reservation-based scheduling.
*   Integration with selected sleep modes and low-power modes.

## References

*   Ralph Lange: Callback-group-level Executor for ROS 2. Lightning talk at ROSCon 2018. Madrid, Spain. Sep 2018. [[Slides]](https://roscon.ros.org/2018/presentations/ROSCon2018_Lightning1_4.pdf) [[Video]](https://vimeo.com/292707644)
* [CB2019] D. Casini, T. Blaß, I. Lütkebohle, B. Brandenburg: Response-Time Analysis of ROS 2 Processing Chains under Reservation-Based Scheduling, in Euromicro-Conference on Real-Time Systems 2019 [[Paper](http://drops.dagstuhl.de/opus/volltexte/2019/10743/)].[[slides]](https://www.ecrts.org/wp-content/uploads/2019/07/casini.pdf)

## Acknowledgments

This activity has received funding from the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement n° 780785).
