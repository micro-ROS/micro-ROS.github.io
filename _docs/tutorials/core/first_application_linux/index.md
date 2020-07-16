---
title: First micro-ROS Application on Linux
permalink: /docs/tutorials/core/first_application_linux/
---

This tutorial teaches you how to create a first micro-ROS application on Linux for testing purposes. In the follow-up tutorial [*First micro-ROS application on an RTOS*](/docs/tutorials/core/first_application_rtos/), you'll learn how to build and bring this application on a microcontroller running the RTOS NuttX, FreeRTOS, or Zephyr.

## Installing ROS 2 and the micro-ROS build system

First of all, install **ROS 2 Dashing Diademata** on your Ubuntu 18.04 LTS computer.
To do so from binaries, via Debian packages, follow the instructions detailed
[here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/) instead.
Alternatively, you can use a docker container with a fresh ROS 2 Dashing installation. The minimum docker that serves
the purpose is the image run by the command:

```bash
docker pull ros:dashing-ros-base
```

```bash
# Source the ROS 2 installation
source /opt/ros/dashing/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

***TIP:** if you are familiar with Docker containers, this image may be useful: [micro-ros:base](https://github.com/micro-ROS/docker/blob/dashing/base/Dockerfile)*

These instructions will setup a workspace with a ready-to-use micro-ROS build system. This build system is in charge of downloading the required cross-compilation tools and building the apps for the required platforms.

The build system's workflow is a four-step procedure:

* **Create step:** This step is in charge of downloading all the required code repositories and cross-compilation toolchains for the specific hardware platform. Among these repositories, it will also download a collection of ready to use micro-ROS apps.
* **Configure step:** In this step, the user can select which app is going to be cross-compiled by the toolchain. Some other options, such as transport, agent address or port will be also selected in this step.
* **Build step:** Here is where the cross-compilation takes place and the platform-specific binaries are generated.
* **Flash step:** The binaries generated in the previous step are flashed onto the hardware platform memory, in order to allow the execution of the micro-ROS app.
Further information about micro-ROS build system can be found [here](https://github.com/micro-ROS/micro-ros-build/tree/dashing/micro_ros_setup).

## Creating a new micro-ROS app

Once the build system is installed, let's create a firmware workspace that targets all the required code and tools:

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh host
```

This step is in charge, among other things, of creating a set of micro-ROS apps for Linux, that are located at
`src/uros/micro-ROS-demos/rcl`. Each app is represented by a folder containing two files: a `.c` file with the app code
and the CMake file. The former contains the logic of the application, whereas the latter contains the script to
compile it.

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the two files just described. Also, any such new application folder needs to be registered in
`src/uros/micro-ROS-demos/rcl/CMakeLists.txt` by adding the following line:

```
export_executable(<my_app>)
```

In this tutorial, we will focus on the out-of-the-box `ping_pong` application located at
`src/uros/micro-ROS-demos/rcl/ping_pong`.
You can check the complete content of this app
[here](https://github.com/micro-ROS/micro-ROS-demos/tree/dashing/rcl/ping_pong).

This example showcases a micro-ROS node with two publisher-subscriber pairs associated with a `ping` and a `pong`
topics, respectively.
The node sends a `ping` package with a unique identifier, using a `ping` publisher.
If the `ping` subscriber receives a `ping` from an external node, the `pong` publisher responds to the incoming `ping`
with a `pong`. To test that this logic is correctly functioning, we implement communication with a ROS 2 node that:

* Listens to the topics published by the `ping` subscriber.
* Publishes a `fake_ping` package, that is received by the micro-ROS `ping` subscriber.
  As a consequence, the `pong` publisher on the micro-ROS application will publish a `pong`, to signal that it received
  the `fake_ping` correctly.

The diagram below clarifies the communication flow between these entities:

![pingpong](http://www.plantuml.com/plantuml/png/ZOwnIWGn48RxFCNFzSkoUG2vqce5jHEHi1dtWZkPa6GByNntavZY10yknMJu-ORlFwPiOjvvK-d3-M2YOR1uMKvHc93ZJafvoMML07d7h1NAE-DPWblg_na8vnwEx9OeZmzFOt1-BK7AzetJciPxCfRYVw1S0SbRLBEg1IpXPIvpUWLCmZpXIm6BS3addt7uQpu0ZQlxT1MK2r0g-7sfqbsbRrVfMrMwgbev3CDTlsqJGtJhATUmSMrMg5TKwaZUxfcttuMt7m00)

The app logic of this demo is contained in
[`app.c`](https://github.com/micro-ROS/micro-ROS-demos/blob/dashing/rcl/ping_pong/main.c).
A thorough review of this file is illustrative of how to create a micro-ROS publisher or subscriber.

Once the app has been created, the build step is in order.
Notice that, with respect to the four-steps workflow delined above, we would expect a configuration step to happen
before building the app. However, given that we are compiling micro-ROS in the host machine rather than in a board,
the cross-compilation implemented by this configure step is not required in this case.
We can then proceed to build the firmware and source the local installation:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```

## Create the micro-ROS agent

The micro-ROS app is now ready to be connected to a micro-ROS agent to start talking with the rest of the ROS 2
world.

To do that, let's first of all create a micro-ROS agent:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```

Now, let's build the agent packages and, when this is done, source the installation:

```bash
# Build step
colcon build
source install/local_setup.bash
```

## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed in your host machine.

To start micro-ROS, first run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

And then, in another command line, run the micro-ROS node (remember sourcing the ROS 2 and micro-ROS installations):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# Run a micro-ROS agent
ros2 run micro_ros_demos_rcl ping_pong
```

## Testing the micro-ROS app

Now, we want to check that everything is working.

Open another command line. We are going to listen to the `ping` topic
with ROS 2 to check whether the micro-ROS Ping Pong node is correctly publishing the expected pings:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS ping topic
ros2 topic echo /microROS/ping
```

You should see the topic messages published by the Ping Pong node every 5 seconds:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 20
  nanosec: 867000000
frame_id: '1344887256_1085377743'
---
stamp:
  sec: 25
  nanosec: 942000000
frame_id: '730417256_1085377743'
---
```

At this point, we know that our app is publishing pings.
Let's check if it also answers to someone else's pings, publishing a pong as an answer.

So, first of all, let's subscribe with ROS 2 to the `pong` topic
(notice that initially we don't expect to receive any pong, since none has been sent yet):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

And now, let's publish a `fake_ping` with ROS 2 from yet another command line:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see this `fake_ping` both in the `ping` subscriber console (the second that we opened),
along with the micro-ROS pings:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
stamp:
  sec: 305
  nanosec: 973000000
frame_id: '451230256_1085377743'
---
stamp:
  sec: 310
  nanosec: 957000000
frame_id: '2084670932_1085377743'
---
```

Also, we expect that, because of having received the `fake_ping`, the micro-ROS `pong` publisher will answer with a
`pong`. As a consequence, in the `pong` subscriber console (the third that we opened),
we should see the micro-ROS app answer to our `fake_ping`:

```
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```

## Multiple Ping Pong nodes

One of the advantages of having an Linux micro-ROS app is that you don't need to buy a bunch of hardware in order to
test some multi-node micro-ROS apps.
So, with the same micro-ROS agent of the last section, let's open four different command lines and run the following on
each:

```bash
cd microros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_demos_rcl ping_pong
```

As soon as all micro-ROS nodes are up and connected to the micro-ROS agent you will see them interacting:

```
user@user$ ros2 run micro_ros_demos_rcl my_brand_new_app
UDP mode => ip: 127.0.0.1 - port: 8888
Ping send seq 1711620172_1742614911                         <---- This micro-ROS node sends a ping with ping ID "1711620172" and node ID "1742614911"
Pong for seq 1711620172_1742614911 (1)                      <---- The first mate pongs my ping
Pong for seq 1711620172_1742614911 (2)                      <---- The second mate pongs my ping
Pong for seq 1711620172_1742614911 (3)                      <---- The third mate pongs my ping
Ping received with seq 1845948271_546591567. Answering.     <---- A ping is received from a mate identified as "546591567", let's pong it.
Ping received with seq 232977719_1681483056. Answering.     <---- A ping is received from a mate identified as "1681483056", let's pong it.
Ping received with seq 1134264528_1107823050. Answering.    <---- A ping is received from a mate identified as "1107823050", let's pong it.
Ping send seq 324239260_1742614911
Pong for seq 324239260_1742614911 (1)
Pong for seq 324239260_1742614911 (2)
Pong for seq 324239260_1742614911 (3)
Ping received with seq 1435780593_546591567. Answering.
Ping received with seq 2034268578_1681483056. Answering.
```
