## Creating the micro-ROS agent

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
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```