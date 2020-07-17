Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of creating a set of micro-ROS apps for the specific platform you are
addressing.
that are located at `src/uros/micro-ROS-demos/rcl`. Each app is represented by a folder containing two files: a `.c`
file with the app code
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

