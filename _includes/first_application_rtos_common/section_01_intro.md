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
