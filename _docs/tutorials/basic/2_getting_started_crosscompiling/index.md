After testing micro-ROS demos running in host computer, this tutorial aims to explain how to implement a demo application in an embedded platform.

These instructions start from a **ROS 2 Dashing** installation and ends with an embedded micro-ROS demo application running on **Crazyflie 2.1** platform using **FreeRTOS**.

## Demo Crazyflie 2.1 + FreeRTOS

The first step is creating a micro-ROS workspace. To do so, clone the **micro-ROS build system** package inside `src` folder and build it: 

```bash
# Source ROS 2 Dashing 
source /opt/ros/dashing/setup.bash

# Create workspace folder
mkdir uros_ws; cd uros_ws;

# Clone micro-ROS build system package
git clone --recursive -b feature/multiboard https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build

# Build micro-ROS build system package
colcon build --packages-select micro_ros_setup

# Source workspace
source install/local_setup.bash
```

Once **micro-ROS build system** package is ready, micro-ROS tools are ready to use. The first step is getting a ready-to-use micro-ROS Agent:

```bash
# Download micro-ROS Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS Agent packages, this may take a while
colcon build
```

Then let's create a environment where build some micro-ROS applications for FreeRTOS running on Crazyflie 2.1:

```bash
# Download micro-ROS host demo packages
ros2 run micro_ros_setup create_firmware_ws.sh freertos crazyflie21

# Configure downloaded packages
ros2 run micro_ros_setup configure_firmware.sh

# Build downloaded packages, this may take a while
ros2 run micro_ros_setup build_firmware.sh
```

Flashing the built firmware on the platform requires enabling DFU mode. To do so, unplug the **Crazyflie 2.1** battery. Then push the reset button while connecting the USB power supply. The top-left blue LED will start blinking, first slowly and after 4 seconds sightly faster, now it is in DFU programming mode. Flash the device using the following command:

```bash
ros2 run micro_ros_setup flash_firmware.sh
```

Once flashing has ended, unplug and plug the **Crazyflie 2.1** power to exit DFU mode.

At this point we have an micro-ROS Agent ready to use and a Crazyflie 2.1 drone flashed with a micro-ROS demo app. It is time to install some platform specific radio software in order to connect the drone with micro-ROS Agent: 

First, install some dependencies:
```bash
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install python3 python3-pip python3-pyqt5 python3-pyqt5.qtsvg
```

Then fix permissions for the Crazyradio PA 2.4 GHz USB dongle (restart required for applying changes):
```bash
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
sudo echo SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", \
MODE=\"0664\", GROUP=\"plugdev\" > /etc/udev/rules.d/99-crazyradio.rules
sudo echo SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", \
MODE=\"0664\", GROUP=\"plugdev\" > /etc/udev/rules.d/99-crazyflie.rules
```

Clone the repo outside of the micro-ROS workspace and run the radio bridge application:
```bash
git clone -b Micro-XRCE-DDS_Bridge https://github.com/eProsima/crazyflie-clients-python
cd crazyflie-clients-python
python3 bin/cfclient
```

This command should open the Crazyflie radio bridge and print a serial device path in the terminal (something like /dev/pts/0). Use this serial port to start a instance of micro-ROS Agent:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev [insert here serial device] -v6
```

Turn on Crazyflie 2.1 drone and click connect on radio bridge. Once it is connected the drone will start publishing its attitude. You can check it using the following command:

```bash 
ros2 topic echo /drone/attitude
```

