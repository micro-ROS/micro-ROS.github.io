---
title: Linux Getting Started
permalink: /docs/tutorials/advanced/linux/linux_getting_started/
---

This tutorial aims to create a new micro-ROS application on Linux for testing purposes.

## Adding a new micro-ROS app

First of all, make sure that you have a **ROS 2** installation.

***TIP:** if you are familiar with Docker containers, this image may be useful: [ros:dashing](https://hub.docker.com/layers/ros/library/ros/dashing/images/sha256-b796c14ea663537129897769aa6c715a851ca08dffd4875ef2ecaa31a4dbd431?context=explore)*

On the **ROS 2** installation open a command line and follow these steps:

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws 
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build

# Update dependencies using rosdep
rosdep update
rosdep install --from-path src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

Now, let's create a firmware workspace that targets all the required code and tools:

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh host
```

Now you have all the required tools to build micro-ROS. At this point, you must know that the micro-ROS build system is a four-step workflow:

<!-- TODO (pablogs9): Remove and link to build-system tutorial when done -->
1. **Create**: retrieves all the required packages for a specific RTOS and hardware platform.
2. **Configure**: configures the downloaded packages with options such as the micro-ROS application, the selected transport layer or the micro-ROS agent IP address (in network transports).
3. **Build**: generates a binary file ready for being loaded in the hardware.
4. **Flash**: load the micro-ROS software in the hardware.

micro-ROS apps for Linux are located at `src/uros/micro-ROS-demos/rcl/`. In order to create a new application, create a new folder containing two files: the app code and the CMake file.

```bash
# Creating a new app
pushd src/uros/micro-ROS-demos/rcl/
mkdir my_brand_new_app
cd my_brand_new_app
touch app.c CMakeLists.cmake
popd
```

For this example we are going to create a ping pong app where a node sends a ping package with a unique identifier using a publisher and the same package is received by a pong subscriber. The node will also answer to pings received from other nodes with a pong message:

![pingpong](http://www.plantuml.com/plantuml/png/ZOwnIWGn48RxFCNFzSkoUG2vqce5jHEHi1dtWZkPa6GByNntavZY10yknMJu-ORlFwPiOjvvK-d3-M2YOR1uMKvHc93ZJafvoMML07d7h1NAE-DPWblg_na8vnwEx9OeZmzFOt1-BK7AzetJciPxCfRYVw1S0SbRLBEg1IpXPIvpUWLCmZpXIm6BS3addt7uQpu0ZQlxT1MK2r0g-7sfqbsbRrVfMrMwgbev3CDTlsqJGtJhATUmSMrMg5TKwaZUxfcttuMt7m00)

To start creating this app, the `CMakeLists.cmake` file should looks like: 

```cmake
cmake_minimum_required(VERSION 3.5)

project(my_brand_new_app LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)

add_executable(${PROJECT_NAME} app.c)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  std_msgs
  rmw_microxrcedds
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
```

Meanwhile `app.c` should look like the following code:

```c
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/header.h>

#include <rmw_uros/options.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define STRING_BUFFER_LEN 100

// Publishing ping guard condition thread
void * trigger_guard_condition(void *args){
  rcl_guard_condition_t * guard_condition = (rcl_guard_condition_t *)args;

  while(true){
    rcl_trigger_guard_condition(guard_condition);
    sleep(5);
  }
}

int main(int argc, const char * const * argv)
{
  //Init RCL options
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&options, rcl_get_default_allocator());
  
  // Init RCL context
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init(0, NULL, &options, &context);

  // Create a node
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_init(&node, "pingpong_node", "", &context, &node_ops);

  // Create a reliable ping publisher
  rcl_publisher_options_t ping_publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t ping_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping", &ping_publisher_ops);

  // Create a best effort pong publisher
  rcl_publisher_options_t pong_publisher_ops = rcl_publisher_get_default_options();
  pong_publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rcl_publisher_t pong_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong", &pong_publisher_ops);

  // Create a best effort  pong subscriber
  rcl_subscription_options_t pong_subscription_ops = rcl_subscription_get_default_options();
  pong_subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rcl_subscription_t pong_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&pong_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong", &pong_subscription_ops);

  // Create a best effort ping subscriber
  rcl_subscription_options_t ping_subscription_ops = rcl_subscription_get_default_options();
  ping_subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rcl_subscription_t ping_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&ping_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping", &ping_subscription_ops);

  // Create guard condition
  rcl_guard_condition_t guard_condition = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_guard_condition_init(&guard_condition, &context, guard_condition_options);

  pthread_t guard_condition_thread;
  pthread_create(&guard_condition_thread, NULL, trigger_guard_condition, &guard_condition);

  // Create a wait set
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_wait_set_init(&wait_set, 2, 1, 0, 0, 0, 0, &context, rcl_get_default_allocator());

  // Create and allocate the pingpong publication message
  std_msgs__msg__Header msg;
  char msg_buffer[STRING_BUFFER_LEN];
  msg.frame_id.data = msg_buffer;
  msg.frame_id.capacity = STRING_BUFFER_LEN;

  // Create and allocate the pingpong subscription message
  std_msgs__msg__Header rcv_msg;
  char rcv_buffer[STRING_BUFFER_LEN];
  rcv_msg.frame_id.data = rcv_buffer;
  rcv_msg.frame_id.capacity = STRING_BUFFER_LEN;

  // Set device id and sequence number;
  int device_id = rand();
  int seq_no;
  
  int pong_count = 0;
  struct timespec ts;
  rcl_ret_t rc;

    do {
    // Clear and set the waitset
    rcl_wait_set_clear(&wait_set);
    
    size_t index_pong_subscription;
    rcl_wait_set_add_subscription(&wait_set, &pong_subscription, &index_pong_subscription);

    size_t index_ping_subscription;
    rcl_wait_set_add_subscription(&wait_set, &ping_subscription, &index_ping_subscription);

    size_t index_guardcondition;
    rcl_wait_set_add_guard_condition(&wait_set, &guard_condition, &index_guardcondition);
    
    // Run session for 100 ms
    rcl_wait(&wait_set, RCL_MS_TO_NS(100));
    
    // Check if some pong message is received
    if (wait_set.subscriptions[index_pong_subscription]) {
      rc = rcl_take(wait_set.subscriptions[index_pong_subscription], &rcv_msg, NULL, NULL);
      if(rc == RCL_RET_OK && strcmp(msg.frame_id.data,rcv_msg.frame_id.data) == 0) {
          pong_count++;
          printf("Pong for seq %s (%d)\n", rcv_msg.frame_id.data, pong_count);
      }
    }

    // Check if some ping message is received and pong it
    if (wait_set.subscriptions[index_ping_subscription]) {
      rc = rcl_take(wait_set.subscriptions[index_ping_subscription], &rcv_msg, NULL, NULL);

      // Dont pong my own pings
      if(rc == RCL_RET_OK && strcmp(msg.frame_id.data,rcv_msg.frame_id.data) != 0){
        printf("Ping received with seq %s. Answering.\n", rcv_msg.frame_id.data);
        rcl_publish(&pong_publisher, (const void*)&rcv_msg, NULL);
      }
    }

    // Check if it is time to send a ping
    if (wait_set.guard_conditions[index_guardcondition]) {
      // Generate a new random sequence number
      seq_no = rand();
      sprintf(msg.frame_id.data, "%d_%d", seq_no, device_id);
      msg.frame_id.size = strlen(msg.frame_id.data);
      
      // Fill the message timestamp
      clock_gettime(CLOCK_REALTIME, &ts);
      msg.stamp.sec = ts.tv_sec;
      msg.stamp.nanosec = ts.tv_nsec;

      // Reset the pong count and publish the ping message
      pong_count = 0;
      rcl_publish(&ping_publisher, (const void*)&msg, NULL);
      printf("Ping send seq %s\n", msg.frame_id.data);  
    }
    
    usleep(10000);
  } while (true);
}
```

Don't forget to register your app in `src/uros/micro-ROS-demos/rcl/CMakeLists.txt` by adding the following line:

```
export_executable(my_brand_new_app)
```

## Running the micro-ROS app

The micro-ROS app is ready to connect to a micro-ROS-Agent and start talking with the rest of the ROS 2 world.

First of all, create a micro-ROS agent:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
```

When the all client and agent packages are ready, just build them all:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```

Then run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Then run the app in another command line (remember sourcing ROS 2 and micro-ROS installation): 

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# Run a micro-ROS agent
ros2 run micro_ros_demos_rcl my_brand_new_app
```

And finally, let's check that everything is working in another command line. We are going to listen to ping topic to check whether the Ping Pong node is publishing its own pings:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS ping topic
ros2 topic echo /microROS/ping
```

You should see the topic messages published by the Ping Pong node every 5 seconds:

```
pgarrido@pgarrido:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 20
  nanosec: 867000000
frame_id: '1344887256_1085377743'
---
stamp:
  sec: 25
  nanosec: 942000000
frame_id: '730417256_1085377743'
---
```

On another command line, let's subscribe to the pong topic

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

At this point, we know that our app is publishing pings. Let's check if it also answers to someone else pings in a new command line:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see on the ping subscriber our fake ping along with the board pings:

```
pgarrido@pgarrido:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
stamp:
  sec: 305
  nanosec: 973000000
frame_id: '451230256_1085377743'
---
stamp:
  sec: 310
  nanosec: 957000000
frame_id: '2084670932_1085377743'
---
```

And in the pong subscriber, we should see the board's answer to our fake ping:

```
pgarrido@pgarrido:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```


## Multiple Ping Pong nodes

One of the advantages of having an Linux micro-ROS app is that you don't need to buy a bunch of hardware in order to test some multi-node micro-ROS apps. So, with the same micro-ROS agent of the last section, let's open four different command lines and run the following on each:

```bash
cd microros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_demos_rcl my_brand_new_app
```

As soon as all micro-ROS nodes are up and connected to the micro-ROS agent you will see them interacting:

```
pgarrido@pgarrido$ ros2 run micro_ros_demos_rcl my_brand_new_app
UDP mode => ip: 127.0.0.1 - port: 8888
Ping send seq 1711620172_1742614911                         <---- This micro-ROS node sends a ping with ping ID "1711620172" and node ID "1742614911"
Pong for seq 1711620172_1742614911 (1)                      <---- The first mate pongs my ping 
Pong for seq 1711620172_1742614911 (2)                      <---- The second mate pongs my ping 
Pong for seq 1711620172_1742614911 (3)                      <---- The third mate pongs my ping 
Ping received with seq 1845948271_546591567. Answering.     <---- A ping is received from a mate identified as "546591567", let's pong it.
Ping received with seq 232977719_1681483056. Answering.     <---- A ping is received from a mate identified as "1681483056", let's pong it.
Ping received with seq 1134264528_1107823050. Answering.    <---- A ping is received from a mate identified as "1107823050", let's pong it.
Ping send seq 324239260_1742614911
Pong for seq 324239260_1742614911 (1)
Pong for seq 324239260_1742614911 (2)
Pong for seq 324239260_1742614911 (3)
Ping received with seq 1435780593_546591567. Answering.
Ping received with seq 2034268578_1681483056. Answering.
```