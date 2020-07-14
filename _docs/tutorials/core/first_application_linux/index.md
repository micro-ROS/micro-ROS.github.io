---
title: First micro-ROS Application on Linux
permalink: /docs/tutorials/core/first_application_linux/
---

This tutorial teaches you how to create a first micro-ROS application on Linux for testing purposes. In the follow-up tutorial [*First micro-ROS application on an RTOS*](/docs/tutorials/core/first_application_rtos/), you'll learn how to build and bring this application on a microcontroller running the RTOS NuttX, FreeRTOS, or Zephyr.

## Installing ROS 2 and the micro-ROS build system

First of all, install **ROS 2 Dashing Diademata** on your Ubuntu 18.04 LTS computer. To do so from sources, please follow the instructions [here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/). To do so from binaries, via Debian packages, please follow the instructions detailed [here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/). Alternatively, you can use a docker container with a fresh ROS 2 Dashing installation, by running the command:

```bash
docker pull ros:dashing
```

Once you have a **ROS 2** installation in the computer, follow these steps to install the micro-ROS build system:

```bash
# Source the ROS 2 installation
source /opt/ros/dashing/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws 
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build

# Install rosdep and vcs
sudo apt install python-rosdep
sudo rosdep init
pip install vcs

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

micro-ROS apps for Linux are located at `src/uros/micro-ROS-demos/rcl/`. In order to create a new application, create a new folder containing two files: the app code and the CMake file. You can check the complete content of these file [here](https://github.com/micro-ROS/micro-ROS-demos/tree/dashing/rcl/ping_pong).

```bash
# Creating a new app
pushd src/uros/micro-ROS-demos/rcl/
mkdir ping_pong
cd ping_pong
touch app.c CMakeLists.txt
popd
```

Don't forget to register your app in `src/uros/micro-ROS-demos/rcl/CMakeLists.txt` by adding the following line:

```
export_executable(ping_pong)
```

For this example we are going to create a ping pong app where a node sends a ping package with a unique identifier using a publisher and the same package is received by a pong subscriber. The node will also answer to pings received from other nodes with a pong message:

![pingpong](http://www.plantuml.com/plantuml/png/ZOwnIWGn48RxFCNFzSkoUG2vqce5jHEHi1dtWZkPa6GByNntavZY10yknMJu-ORlFwPiOjvvK-d3-M2YOR1uMKvHc93ZJafvoMML07d7h1NAE-DPWblg_na8vnwEx9OeZmzFOt1-BK7AzetJciPxCfRYVw1S0SbRLBEg1IpXPIvpUWLCmZpXIm6BS3addt7uQpu0ZQlxT1MK2r0g-7sfqbsbRrVfMrMwgbev3CDTlsqJGtJhATUmSMrMg5TKwaZUxfcttuMt7m00)

The app logic of this demo is contained in  [`app.c`](https://github.com/micro-ROS/micro-ROS-demos/blob/dashing/rcl/ping_pong/main.c). A thorough review of this file can show how to create a micro-ROS publisher or subscriber, as shown below. 

**For more in depth learning about RCL and RCLC programming [check this section](https://micro-ros.github.io/docs/tutorials/core/programming_rcl_rclc/).**
```c
...
  // Create a reliable ping publisher
  rcl_publisher_options_t ping_publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t ping_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping", &ping_publisher_ops);
...

...
  // Create a best effort  pong subscriber
  rcl_subscription_options_t pong_subscription_ops = rcl_subscription_get_default_options();
  pong_subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rcl_subscription_t pong_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&pong_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong", &pong_subscription_ops);
...
... 
    // Check if some pong message is received
    if (wait_set.subscriptions[index_pong_subscription]) {
      rc = rcl_take(wait_set.subscriptions[index_pong_subscription], &rcv_msg, NULL, NULL);
      if(rc == RCL_RET_OK) {
          // Subscription app logic
      }
    }

...
    // Reset the pong count and publish the ping message
    rcl_publish(&ping_publisher, (const void*)&msg, NULL);
...
```

## Running the micro-ROS app

The micro-ROS app is ready to be built and connected to a micro-ROS-Agent to start talking with the rest of the ROS 2 world.

First of all, create a micro-ROS agent:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```

When the all client and agent packages are ready, just build them all:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```

Then run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Then run the app in another command line (remember sourcing ROS 2 and micro-ROS installation): 

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# Run a micro-ROS agent
ros2 run micro_ros_demos_rcl ping_pong
```

And finally, let's check that everything is working in another command line. We are going to listen to ping topic to check whether the Ping Pong node is publishing its own pings:

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

On another command line, let's subscribe to the pong topic

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

At this point, we know that our app is publishing pings. Let's check if it also answers to someone else pings in a new command line:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see on the ping subscriber our fake ping along with the board pings:

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

And in the pong subscriber, we should see the board's answer to our fake ping:

```
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```


## Multiple Ping Pong nodes

One of the advantages of having an Linux micro-ROS app is that you don't need to buy a bunch of hardware in order to test some multi-node micro-ROS apps. So, with the same micro-ROS agent of the last section, let's open four different command lines and run the following on each:

```bash
cd microros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_demos_rcl my_brand_new_app
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
