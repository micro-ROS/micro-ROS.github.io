This tutorial is an entry point for micro-ROS application development on **ROS2 Dashing**.

**micro-ROS build system** is an utility package that contains all necessary tools to cross compile applications on embedded platforms.

Detailed information about this build system and supported platforms can be found in the [package repo](https://github.com/micro-ROS/micro-ros-build/tree/feature/multiboard/micro_ros_setup).

## Simple publisher and subscriber

A simple example of an *int32* publisher and subscriber is going to be implemented along this tutorial. As is exposed in the build system documentation, the starting point for this tutorial is a **ROS 2 Dashing** installation.

This micro-ROS example will use your local computer (aka host) as platform. This way, both nodes and agent will run in your host. Detailed information about micro-ROS architecture can be found [here](https://micro-ros.github.io/docs/overview/).

Let's start creating a micro-ROS workspace. Clone the **micro-ROS build system** package inside `src` folder and build it:

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

Then, let's create a environment where build some host demo micro-ROS applications. Download, configure and build some demo apps:

```bash
# Download micro-ROS host demo packages
ros2 run micro_ros_setup create_firmware_ws.sh host

# Configure downloaded packages
ros2 run micro_ros_setup configure_firmware.sh

# Build downloaded packages, this may take a while
ros2 run micro_ros_setup build_firmware.sh
```

Now all required micro-ROS packages are built, so it is possible to run the demos. You will need three different command lines in order to run elements separately:

```bash
# REMEMBER: Source workspaces on all command lines
source /opt/ros/dashing/setup.bash
source install/local_setup.bash

# Command line 1: Run an UDP micro-ROS Agent on port 8888 with highest verbosity level
ros2 run micro_ros_agent micro_ros_agent udp --port 8888 -v6

# Command line 2: Run an int32 subscriber
RMW_IMPLEMENTATION=rmw_microxrcedds ros2 run micro_ros_demos_rcl int32_subscriber

# Command line 3: Run an int32 publisher
RMW_IMPLEMENTATION=rmw_microxrcedds ros2 run micro_ros_demos_rcl int32_publisher
```

You should see how the publisher and the subscriber register all their entities on the micro-ROS Agent. Then the publisher node will publish integers up to 1000 and the subscriber node should receive them.

### Code analysis

The demos' source code is located in `uros_ws/src/uros/micro-ROS-demos/rcl/int32_publisher` and `uros_ws/src/uros/micro-ROS-demos/rcl/int32_subscriber` respectively. 

A brief explanation about how this micro-ROS nodes are implemented using RCL API is presented. Further information about this ROS 2 Client Library can be found [here](http://docs.ros2.org/dashing/developer_overview.html#the-rcl-repository).

First, RCL options and context structs are instantiated and initialized:

```c
rcl_init_options_t options = rcl_get_zero_initialized_init_options();
rcl_init_options_init(&options, rcl_get_default_allocator());

rcl_context_t context = rcl_get_zero_initialized_context();
rcl_init(argc, argv, &options, &context);
```

Secondly, a node is created using RCL API:

```c
rcl_node_options_t node_ops = rcl_node_get_default_options();
rcl_node_t node = rcl_get_zero_initialized_node();

rcl_node_init(&node, "int32_node", "", &context, &node_ops);
```

Then, a publisher or a subscriber are initialized:

```c
// Creating a publisher
rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &publisher_ops);

// Creating a subscriber
rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &subscription_ops);
```

At this point, is possible to publish a message on a topic as seen in `int32_publisher` demo code:

```c
std_msgs__msg__Int32 msg;
msg.data = 42;
rcl_publish(&publisher, (const void*)&msg, NULL);
```

In order to get a subscriber ready some other steps are required. First, it is necessary to create a wait set. This element holds ROS 2 related elements such as subscriptions, services or client, and handles their state.

```c
rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();

// Init wait_set with 1 subscription
rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
```

To check if subscription has received data the wait set has to be prepared and a RCL wait function must be called:

```c
// Clear wait set
rcl_wait_set_clear(&wait_set);

// Add the subscription to the wait set
rcl_wait_set_add_subscription(&wait_set, &subscription, NULL);

// Run middleware during 1 second
rcl_wait(&wait_set, RCL_MS_TO_NS(1000));
```

Once the wait has ended, it is possible to check the wait set subscription status and retrieve subscription data if it is ready.

```c
if(wait_set.subscriptions[0] != NULL){ // If not NULL the subscription has data
    std_msgs__msg__Int32 msg;

    // Retrieve the available data in the message struct
    if (RCL_RET_OK == rcl_take(wait_set.subscriptions[i], &msg, NULL, NULL)){
        printf("I received: [%i]\n", msg.data);
    }
}
```