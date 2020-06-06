Once the command is executed, a folder named `firmware` must be present in your workspace.

## Step 2: Configuring the firmware

The configuration step will set up the main micro-ROS options and will select the required application. It can be executed with the following command:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
```

The options available for this configuration step are:
  - `--transport` or `-t`: `udp`, `tcp`, `serial` or any hardware-specific transport label
  - `--dev` or `-d`: agent string descriptor in a serial-like transport
  - `--ip` or `-i`: agent IP in a network-like transport
  - `--port` or `-p`: agent port in a network-like transport

At this point, in order to build your first micro-ROS application you can use the following example as a reference:

