# MoveIt 2 + micro-ROS demo

This demo shows the integration between micro-ROS and [MoveIt 2](https://moveit.ros.org/), a manipulation framework for robotics applications created and maintained by [PickNik](https://picknik.ai/). 

By running this demo code, you can see how the MoveIt 2 manipulation and planification algorithms are fed by the position (attitude) calculated in a pose estimator that runs in micro-ROS. This pose is calculated by using data from inertial sensors. The final result is displayed with the ROS visualization tool RViz.

micro-ROS runs in a a [STM32L4 Development IoT kit](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) board in combination with [Zephyr](https://zephyrproject.org/). The board offers several general-purpose I/O pins and peripherals to communicate its 32-bits microcontroller with the external world. It also includes a lot of sensors. For this demo, we make use of a 6-DoF Inertial Measurement Unit (LSM6DSL), composed of an accelerometer and a gyroscope, and a 3-DoF magnetometer (LIS3MDL). The fusion of the measurements fetched by these sensors outputs the pose, or relative orientation of the board with respect to a fixed reference frame.

The pose data is then forwarded to the ROS 2 world, where it is consumed by both RViz and MoveIt 2. RViz uses it directly to represent the position and orientation of the board in its graphical interface, and MoveIt 2 uses it to calculate the movement that the virtual arm has to perform to ‘touch’ it, according to its kinematic algorithms. The resulting movement is then integrated into RViz and represented by means of its virtual panda robotic arm, a standard tool used by MoveIt in tutorials and graphic interfaces.

![Demo setup](https://raw.githubusercontent.com/micro-ROS/micro-ROS_moveit2_demo/foxy/images/demo.png)

## Usage

### micro-ROS

Install and run micro-ROS attitude_estimator demo for ST Discovery board and Zephyr RTOS. 
For detailed info about the micro-ROS build system visit [micro-ROS tutorials](https://micro-ros.github.io/docs/tutorials/core/first_application_linux/)

```bash
# Create a micro-ROS Agent 
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

# Build and flash the micro-ROS app
ros2 run micro_ros_setup create_firmware_ws.sh zephyr discovery_l475_iot1
# Check https://github.com/micro-ROS/zephyr_apps/tree/foxy/apps/attitude_estimator for instructions for tf2_msgs
ros2 run micro_ros_setup configure_firmware.sh attitude_estimator -t serial -d 1
ros2 run micro_ros_setup build_firmware.sh

# Connect the ST Discovery board with the ST-Link USB port and switch the power selector to the correct position
ros2 run micro_ros_setup flash_firmware.sh

# Run the micro-ROS Agent
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev [ST Disco serial device] -v6

```

### MoveIt2

Using a ROS 2 Foxy installation install MoveIt2 as explained in [their webpage](https://moveit.ros.org/install-moveit2/source/).

Now in the same workspace:

```bash
git clone https://github.com/micro-ROS/micro-ROS_moveit2_demo
colcon build --event-handlers desktop_notification- status- --packages-select microros_moveit2_demo --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash

ros2 launch microros_moveit2_demo microros_moveit2_demo.launch.py
```
