---
title: Crazyflie Demo
permalink: /docs/tutorials/demos/crazyflie_demo/
redirect_from:
  - /crazyflie_demo/
---

# Crazyflie + Turtlebot demo

This demo aims to expose an use case of **Micro ROS** running on a pair or embedded devices: a [**Crazyflie 2.1**](https://www.bitcraze.io/crazyflie-2-1/) drone, used as an user controller and a [**Kobuki Turtlebot 2**](https://www.turtlebot.com/turtlebot2/) as a mobile and controlled device.

Both of them rely on **Micro ROS** publication and subscription mechanisms and use an underlaying [**MicroXRCEDSS client**](https://micro-xrce-dds.readthedocs.io/en/latest/).

In order to show the integration with **ROS2**, this demo also includes common ROS2 tooling such as Gazebo, RVIZ and simple ROS2 nodes (aka **external nodes**) acting as data converters.

## Index

- [Crazyflie + Turtlebot demo](#crazyflie--turtlebot-demo)
  - [Index](#index)
  - [Setup](#setup)
  - [Required Hardware](#required-hardware)
- [Installation](#installation)
  - [Install external ROS2 nodes](#install-external-ros2-nodes)
  - [Compile and flash Crazyflie 2.1 firmware](#compile-and-flash-crazyflie-21-firmware)
  - [Install Crazyflie Client + Bridge](#install-crazyflie-client--bridge)
  - [Compile and flash Kobuki Turtlebot 2 firmware](#compile-and-flash-kobuki-turtlebot-2-firmware)
- [Usage](#usage)
  - [Run Kobuki Turtlebot 2 Node](#run-kobuki-turtlebot-2-node)
  - [Run Crazyflie 2.1 Node](#run-crazyflie-21-node)
  - [Run external ROS2 nodes](#run-external-ros2-nodes)
  - [Run RVIZ visualizers](#run-rviz-visualizers)

## Setup

The proposed demo is composed by different kind of messages and topics. 

The **Crazyflie 2.1** drone relies on [ST STM32F405](https://www.st.com/en/microcontrollers-microprocessors/stm32f405-415.html) MCU running **[FreeRTOS](https://www.freertos.org/)**. Using the RTOS capabilities and the integrated radio communication device, the drone is able to run a node that publish:
- its own relative position as a 3D vector (X, Y and Z) using a *geometry_msg/Point32* message type on */drone/odometry* topic.
- its own attitude as a 3D vector (pitch, roll and yaw) using a *geometry_msg/Point32* message type on */drone/attitude* topic.

The **Kobuki Turtlebot 2** robot is controlled using a UART protocol through a custom DB25 connector. The Micro ROS node runs on a Olimex STM32-E407 board attached to that UART port. This hardware features a [ST STM32F407](https://www.st.com/en/microcontrollers-microprocessors/stm32f407-417.html) MCU running **[Nuttx](https://nuttx.org/)** RTOS. In the same way, this node is able to communicate with the robot (UART) and with the ROS2 world (integrated Ethernet). Its used topics are:
- a subscription on */cmd_vel* topic (*geometry_msg/Twist* message type) to receive the controlling angular and linear velocity.
- a publication on */robot_pose* topic (*geometry_msg/Vector3* message type) which includes X position, Y position and robot yaw. 

The **external ROS2 nodes** are rclpy tools with some different functionalities:
- *attitude_to_vel.py*
  - Converts Crazyflie */drone/attitude* to Kobuki Turtlebot 2 */cmd_vel* so that drone pitch is mapped to robot linear velocity and drone roll to angular valocity.
  - Converts Crazyflie publications on */drone/attitude* and */drone/attitude* topics to *tf2_msgs/TFMessage* messages (required by RVIZ visualizer)
- *odom_to_tf.py*
  - Converts Kobuki Turtlebot 2 publications on */robot_pose* topic to *tf2_msgs/TFMessage* messages (required by RVIZ visualizer).

The following image shows the described setup.

![Kobuki_drone_demo](http://www.plantuml.com/plantuml/png/XLDHJzim47xlhp3P2wlMP0FniWS40uhGDeegreUXANAJQx18V95zBcBJ_llYIpChfDdwuChVz_axt-VBcILfo5Nbv227ZT8WRYuMjz-MNyGZKMq_9ecHpt6XwD6jdGMJeIRG56TO9UHgcPlZf2wbzXOprR2pJQEOsTee0fjiZ-8FyVl9WT9PwN9mfkpyayQXGXtNN7iFppxo6InMC3j93AwHnjK5Rdrrc-G6DRGw-wHqBOsin5fcJuL1f_CBBD68D_FvV38HpKzf0hEH6OYeFTeMIckq81xkiE6FZtw8wVJAbM34kIvAiDDf9AGL30rSiYfFjr2AXmAm0Z8lQMKBcpHBSl-iB7cp5PIOANhP6J4-C0eNsUUrWepG77ktHSvavzPjH_h37TthxWwj8eLwPz5jsQ8DwdgnIY-NYzkhmqjlyqv4_1-zPVO2gxhPQHAII97B8INqCSJro-IL8hgMFs6DuZEktPEAX2_OGcaB2JumFpz9bujFY_l35cqgxawq9Nana97qRoAYhR9EbifAV_58_6B-TGIq4G_tywzWhInWJ-D_kS7f460jwR49hrdPmC1MeGkPTTFX1UpIxzx7xxEXZOzco4VBrSrtmTcAyrsMxEVnbF5K_kTSTvNNv-hHSssoGT_kMVuxfDswpsQdOVG35eQ6SLQ8E3xTCn4iwEJ_qWnXhXI-bp51UCPHKNWWNjYJxn9w3s1_GBd3FiIElIzl7yv4jsFV_ZWm7w11Eom8MtRelzUG3LCpIeVXu526p1det5Nb7m00)

## Required Hardware

This setup uses the following hardware:

| Item          |                                                          |
|---------------|----------------------------------------------------------|
| Kobuki Turtlebot 2 | [Link](https://www.turtlebot.com/turtlebot2/) |
| Olimex STM32-E407 | [Link](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) |
| Olimex ARM-USB-TINY-H | [Link](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/) |
| Crazyflie 2.1 | [Link](https://store.bitcraze.io/products/crazyflie-2-1) |
| Flow Desk v2  | [Link](https://store.bitcraze.io/collections/decks/products/flow-deck-v2) |
| Debug adapter  | [Link](https://store.bitcraze.io/collections/accessories/products/debug-adapter) |
| Crazyradio PA 2.4 GHz USB dongle | [Link](https://store.bitcraze.io/collections/accessories/products/crazyradio-pa) |
| Additional battery + charger (optional) | [Link](https://store.bitcraze.io/collections/accessories/products/240mah-lipo-battery-including-500ma-usb-charger) |


# Installation

## Install external ROS2 nodes

[Install MicroXCRE-DDS](https://micro-xrce-dds.readthedocs.io/en/latest/installation.html). Recommended procedure:

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS.git
cd Micro-XRCE-DDS
mkdir build && cd build
cmake ..
make
sudo make install
```

Create a workspace folder for this demo:
```bash
mkdir -p crazyflie_demo/src
cd crazyflie_demo
```

Clone this repo:
```bash
git clone --single-branch --branch crazyflie_demo https://github.com/micro-ROS/micro-ROS_kobuki_demo src
```

[Install Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install#InstallGazebousingUbuntupackages). Recommended procedure:
```bash
curl -sSL http://get.gazebosim.org | sh
```

[Install gazebo_ros_pkgs (ROS 2)](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros). Recommended procedure:
```bash
source /opt/ros/dashing/setup.bash
wget https://bitbucket.org/api/2.0/snippets/chapulina/geRKyA/f02dcd15c2c3b83b2d6aac00afe281162800da74/files/ros2.yaml
vcs import src < ros2.yaml
rosdep update && rosdep install --from-paths src --ignore-src -r -y
rm ros2.yaml
```

Compile the project:
```bash
source /opt/ros/dashing/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Compile and flash Crazyflie 2.1 firmware

Install the toolchain:
```bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt install gcc-arm-embedded dfu-util
```

Download and copile the **Crazyflie 2.1** firmware repository:
```bash
mkdir crazyflie_firmware
git clone https://github.com/eProsima/crazyflie-firmware -b cf_micro-xrce-dds 
cd crazyflie_firmware
git submodule init
git submodule update
make PLATFORM=cf2
```

Unplug the **Crazyflie 2.1** battery

Push the reset button while connecting the USB power supply.

The top-left blue LED will blink, first slowly and after 4 seconds sightly faster, now it is in DFU programming mode. Check it with `lsusb`:
```bash
Bus 001 Device 051: ID 0483:df11 STMicroelectronics STM Device in DFU Mode
```

Flash the device:
```bash
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D cf2.bin
```

Unplug and plug the **Crazyflie 2.1** power to exit DFU mode.


## Install Crazyflie Client + Bridge

Install dependencies:
```
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install python3 python3-pip python3-pyqt5 python3-pyqt5.qtsvg
```

Fix permissions for the Crazyradio PA 2.4 GHz USB dongle (restart required for apply changes):
```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
sudo echo SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", \
MODE=\"0664\", GROUP=\"plugdev\" > /etc/udev/rules.d/99-crazyradio.rules
sudo echo SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", \
MODE=\"0664\", GROUP=\"plugdev\" > /etc/udev/rules.d/99-crazyflie.rules
```

Clone the repo dependencies:
```
git clone -b Micro-XRCE-DDS_Bridge https://github.com/eProsima/crazyflie-clients-python 
```

## Compile and flash Kobuki Turtlebot 2 firmware

Create a workspace for compiling **Micro ROS**:
```
source /opt/ros/crystal/setup.bash
sudo apt install python-rosdep
mkdir -p kobuki-firmware/src
cd kobuki-firmware
git clone --recursive -b crazyflie_demo https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build
colcon build --packages-select micro_ros_setup
source install/local_setup.bash
```

Build **Micro ROS Agent**:
```
colcon build --packages-select micro_ros_setup
source install/local_setup.bash
```

Build Olimex STM32-E407 firmware:
```
ros2 run micro_ros_setup create_firmware_ws.sh
cd firmware/NuttX
tools/configure.sh configs/olimex-stm32-e407/drive_base # FOR EXAMPLE!
cd ../..
ros2 run micro_ros_setup build_firmware.sh
 ```

Connect Olimex ARM-USB-TINY-H JTAG debugger to Olimex STM32-E407 and flash the board:
```
cd firmware/NuttX
scripts/flash.sh olimex-stm32-e407
```

# Usage

After installation, the following packages should be installed:

```
.
+-- Micro-XRCE-DDS          # used for installing Micro-XRCE-DDS
+-- crazyflie_demo 
+-- crazyflie-firmware      # used for compiling and flashing Crazyflie 2.1 firmware
+-- kobuki-firmware         # used for compiling and flashing Kobuki Turtlebot 2 firmware
+-- crazyflie-clients-python
```

Make sure that all ROS2 or MicroROS nodes created along the following steps **can reach each other using its network interfaces**.  

## Run Kobuki Turtlebot 2 Node

TODO: Explain data and power connections between Kobuki Turtlebot 2, Olimex STM32-E407 and MiniRouter.

Run the **Micro ROS Agent**:
```
cd kobuki-firmware
source /opt/ros/crystal/setup.bash && source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp 9999
```

**Micro ROS Agent** should receive an incoming client connection and */robot_pose* topic should be published. Check it with `ros2 topic echo /robot_pose`

## Run Crazyflie 2.1 Node

Connect Crazyradio PA 2.4 GHz USB dongle and turn on Crazyflie 2.1 drone.

Run the Crazyflie Client + Bridge:
```
cd crazyflie-clients-python
python3 bin/cfclient
```

This command should open the Crazyflie Client and print a serial device path in the terminal (something like /dev/pts/0).

Run (in another prompt) a **MicroXRCE Agent**:
```
MicroXRCEAgent serial --dev [serial device]
```

**MicroXRCE Agent** should receive an incoming client connection and */drone/attitude* and */drone/position* topics should be published. Check it with `ros2 topic echo /drone/attitude` and `ros2 topic echo /drone/position`


## Run external ROS2 nodes

Run commands:
```
cd crazyflie_demo
source /opt/ros/crystal/setup.bash && source install/local_setup.bash
ros2 run micro-ros_crazyflie_demo_remote attitude_to_vel
```

Topic */cmd_vel* should be published and the **Kobuki Turtlebot 2** should start moving. Check it with `ros2 topic echo /cmd_vel`

## Run RVIZ visualizers
Run complete visualizer:
```
cd crazyflie_demo
source /opt/ros/crystal/setup.bash && source install/local_setup.bash
ros2 launch micro-ros_crazyflie_demo_remote launch_drone_position.launch.py
```

An RVIZ windows should be open and a Crazyflie 2.1 drone model should represent the drone attitude and position along with a historic path.

Run attitude visualizer:
```
cd crazyflie_demo
source /opt/ros/crystal/setup.bash && source install/local_setup.bash
ros2 launch micro-ros_crazyflie_demo_remote launch_drone_attitude.launch.py
```

An RVIZ windows should be open and a Crazyflie 2.1 drone model should represent **only** the drone attitude.


