---
title: First micro-ROS Application on Zephyr
permalink: /docs/tutorials/core/first_application_rtos/zephyr/
redirect_from:
  - /docs/tutorials/advanced/zephyr/zephyr_getting_started/
---

## Target platform

In this tutorial, you'll learn the use of micro-ROS with Zephyr, by testing a Ping Pong application.

The target hardware for this tutorial is the
**[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)** evaluation board.

The following hardware will be used:

* [Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
* [Olimex ARM-USB-TINY-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/)
* [USB-Serial Cable Female](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-Serial-Cable-F/)

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

***TIP:** if you are familiar with Docker containers, this image may be useful:
[micro-ros:base](https://github.com/micro-ROS/docker/blob/dashing/base/Dockerfile)*

These instructions will setup a workspace with a ready-to-use micro-ROS build system.
This build system is in charge of downloading the required cross-compilation tools and building the apps for the
required platforms.

The build system's workflow is a four-step procedure:

* **Create step:** This step is in charge of downloading all the required code repositories and cross-compilation
  toolchains for the specific hardware platform. Among these repositories, it will also download a collection of ready
  to use micro-ROS apps.
* **Configure step:** In this step, the user can select which app is going to be cross-compiled by the toolchain.
  Some other options, such as transport, agent address or port will be also selected in this step.
* **Build step:** Here is where the cross-compilation takes place and the platform-specific binaries are generated.
* **Flash step:** The binaries generated in the previous step are flashed onto the hardware platform memory, in order
  to allow the execution of the micro-ROS app.

Further information about micro-ROS build system can be found
[here](https://github.com/micro-ROS/micro-ros-build/tree/dashing/micro_ros_setup).

## Step 1: Creating a new firmware workspace

Once the build system is installed, let's create a firmware workspace that targets all the required code and tools:

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh zephyr olimex-stm32-e407
```

**NOTE for Zephyr: Make sure you have the latest version of CMake!**

```bash
sudo apt install wget
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt install software-properties-common
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt update
sudo apt install cmake
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of creating a set of micro-ROS apps for the specific platform you are
addressing. In the case of Zephyr, these are located at `firmware/zephyr_apps/apps`.
Each app is represented by a folder containing the following files:

* `src/main.c`: This file contains the logic of the application.
* `app-colcon.meta`: This file contains the micro-ROS app specific colcon configuration. Detailed info on how to
  configure the RMW via this file can be found
  [here](https://micro-ros.github.io/docs/tutorials/core/microxrcedds_rmw_configuration/).
* `CMakeLists.txt`: This is the CMake file containing the script to compile the application.
* `prj.conf`: This is a Zephyr specific app configuration file.

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the four files just described.

In this tutorial, we will focus on the out-of-the-box `ping_pong` application located at
`firmware/zephyr_apps/apps/ping_pong`.
You can check the complete content of this app
[here](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong).

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

The contents of the Zephyr app specific files can be found here:
[main.c](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/src/main.c),
[app-colcon.meta](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/app-colcon.meta),
[CMakeLists.txt](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/CMakeLists.txt)
and [prj.conf](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/prj.conf).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

Once the app has been created, the configuration step is in order.

## Step 2: Configuring the firmware

The configuration step will set up the main micro-ROS options and will select the required application.
It can be executed with the following command:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
```

The options available for this configuration step are:
  - `--transport` or `-t`: `udp`, `tcp`, `serial` or any hardware-specific transport label
  - `--dev` or `-d`: agent string descriptor in a serial-like transport
  - `--ip` or `-i`: agent IP in a network-like transport
  - `--port` or `-p`: agent port in a network-like transport

At this point, in order to build and try your first micro-ROS application, you can use the out-of-the-box `ping_pong`
application located at `firmware/zephyr_apps/apps/ping_pong`. You can check the complete content of this app
[here](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong). To execute it, run the command above
by specifying the `[APP]` and `[OPTIONS]` parameters as below:

```bash
# Configure step with ping_pong app and serial-usb transport
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial-usb
```

## Step 3: Building the firmware

When the configuring step ends, just build the firmware and source the local installation:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```

## Step 4: Flashing the firmware

Flashing the firmware into the platform varies across hardware platforms.
Regarding this tutorial's target platform
(**[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)**),
the JTAG interface is going to be used to flash the firmware.

Connect [Olimex ARM-USB-TINY-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/) to the board:

<img width="400" style="padding-right: 25px;" src="../imgs/2.jpg">

Make sure that the board power supply jumper (PWR_SEL) is in the 3-4 position in order to power the board from the
JTAG connector:

<img width="400" style="padding-right: 25px;" src="../imgs/1.jpg">

Once you have your computer connected to the Olimex board through the JTAG adapter, run the flash step:

```bash
# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```

## Running the micro-ROS app

The micro-ROS app is ready to connect to a micro-ROS-Agent and start talking with the rest of the ROS 2 world.

First of all, create and build a micro-ROS agent:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS-Agent packages, this may take a while.
colcon build
source install/local_setup.bash
```

Then, depending on the selected transport and RTOS, the board connection to the agent may differ:

```



==========================








{% include first_application_rtos_common/section_03_configuring_firmware.md %}

{% include first_application_rtos_common/section_04_demo_description.md %}

micro-ROS apps for Zephyr are located at `firmware/zephyr_apps/apps`. In order to create a new application, create a new folder containing two files: the app code (inside a `src` folder) and the RMW configuration. You will also need some other Zephyr related files: a `CMakeLists.txt` to define the building process and a `prj.conf` where Zephyr is configured. There is a sample proyect [here](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong), for now, it is ok to copy them.

Create a new app:

```bash
# Create your app folder and required files. Contents of these file can be found in column Sample app in table above
pushd firmware/zephyr_apps/apps
mkdir ping_pong
cd ping_pong
mkdir src
touch src/app.c app-colcon.meta
touch CMakeLists.txt prj.conf
popd
```

The contents of the files can be found here: [app.c](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/src/main.c), [app-colcon.meta](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/app-colcon.meta), [CMakeLists.txt](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/CMakeLists.txt) and [prj.conf](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/prj.conf).

Once the app folder is created, let's configure our new app with a UDP transport that looks for the agent on the port UDP/8888 at localhost:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial-usb
```






{% include first_application_rtos_common/section_05_building_flashing_and_running.md %}

|    RTOS    | micro-ROS Client to Agent |
| :--------: | ------------------------- |
|   NuttX    | Serial                    |
|  FreeRTOS  | Serial                    |
| **Zephyr** | **USB**                   |

{% include first_application_rtos_common/section_06_agent.md %}

This completes the First micro-ROS Application on Zephyr tutorial.
Do you want to [go back](../) and try a different RTOS, i.e. NuttX or FreeRTOS?
