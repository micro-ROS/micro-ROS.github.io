## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed.

To start micro-ROS, you just need to run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

***TIP:** you can use this command to find your serial device name: `ls /dev/serial/by-id/*`*