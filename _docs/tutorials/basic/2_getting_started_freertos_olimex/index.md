This tutorial aims to setup a Hello World micro-ROS application on **[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)** evaluation board with **[FreeRTOS RTOS](https://www.freertos.org/)**


*TODO: Link to micro-ROS explanation (e.g Agent)*

*TODO: Image of the board and FreeRTOS logo*

## Required hardware

This tutorial uses the following hardware:

| Item | |
|---------------|----------------------------------------------------------|
| Olimex STM32-E407 | [Link](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) |
| Olimex ARM-USB-TINY-H | [Link](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/) |
| USB-Serial Cable Female | [Link](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-Serial-Cable-F/) |


Olimex STM32-E407 features a STM32F407 Cortex-M4 microcontroller. It has 1MB Flash and 196 kB RAM of which 64 kB are core coupled (CCM SRAM). Other features of the chip are:

- Three 12 bits @ 2.4MHz ADC
- Two 12 bits DAC
- USB OTG FS
- USB OTG HS
- Ethernet
- 14 timers
- 3 SPI
- 3 I2C
- 2 CAN
- 114 GPIOs

Most of this peripherals are mapped into Olimex board headers.

The micro-ROS + FreeRTOS port of this evaluation board relies on a STM32CubeMX generated project, so HAL and low-level configuration can be tunned to specific requirements.

The out-of-the-box HAL configuration included in the port packages configures the minimal communication and debugging peripheral required for micro-ROS.

## Running micro-ROS

First of all make sure that you have a **ROS 2 Dashing** installation.

***TIP:** if you are familiar with Docker containers, this image may be useful: [ros:dasing](https://hub.docker.com/layers/ros/library/ros/dashing/images/sha256-b796c14ea663537129897769aa6c715a851ca08dffd4875ef2ecaa31a4dbd431?context=explore)*

On the **ROS 2 Dashing** installation open a command line and follow this steps:

```bash
# Source the ROS 2 Dashing installation
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

Now you have all the requiered tools to crosscompile micro-ROS and FreeRTOS for Olimex STM32-E407 development board. At this point, you must know that the micro-ROS build system is a four-step workflow:

1. **Create**: retrieve all the required packages for a specific RTOS and hardware platform.
2. **Configure**: configures the downloaded packages with options such as the micro-ROS application, the selected transport layer or the micro-ROS agent IP address (in network transports).
3. **Build**: generate a binary file ready for being loaded in the hardware.
4. **Flash**: load the micro-ROS software in the hardware.

So, lets create a firmware workspace for the target system:

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh freertos olimex-stm32-e407
```

Once we have the firmware packages ready, lets configure the app to the `simple_publisher` sample and a serial transport on UART3.

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh olimex_simple_publisher --transport serial --dev 3
```

You can check the sample app [code here](https://github.com/micro-ROS/freertos_apps/blob/dashing/apps/olimex_simple_publisher/app.c).

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

Once the build has successfully ended, lets power and connect the board. 

First, connect Olimex ARM-USB-TINY-H JTAG programmer to the board's JTAG port:

*TODO: JTAG image*

Make sure that the board power supply jumper (PWR_SEL) is in the 3-4 position in order to power the board from the JTAG connector:

*TODO: jumper image*

```bash
# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```

## Test the app

The micro-ROS app is ready to connect to a micro-ROS Agent and start talking with the rest of the ROS 2 world.

First of all, create and build a micro-ROS agent:

```bash
# Download micro-ROS Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS Agent packages, this may take a while.
colcon build
```

Then connect the Olimex development board to the computer using the usb to serial cable:

*TODO: usb serial image*

Then run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

***TIP:** you can use this command to find your serial device name: `ls /dev/serial/by-id/*`*

And finally, check that everything is working:

```bash
# Subscribe to micro-ROS sample app topic
ros2 topic echo /olimex/ping_publisher
```

## Adding a new micro-ROS app

Micro-ROS apps for Olimex + FreeRTOS are located at `firmware/freertos_apps/apps`. In order create a new application a new folder containing two files: the app code and the RMW configuration.


```bash
# Creating a new app
cd firmware/freertos_apps/apps
touch app.c app-colcon.meta
```

For this example we are going to create a ping pong app where a node sends a ping package with a unique identifier using a ping_publisher and receives a pong package on a subscriber.

![pingpong](http://www.plantuml.com/plantuml/png/bOwnIWGn48RxFCNFzSkoUG2vqce5jHEHi1dtWZkPa6GByNntavZY40yqnMJu-ORlFwPiOjvvK-dD-M2YOR1uMKvHc93ZJafvoMML07d5h1NAE-DHWblg_nu8vnwEx9Oem_tTmnYSNqkIidtXjARnbeob-2ifLqZo5jMjQWLBU9hBd9u1ap1F-5308bpEoMSSVhWFG0FhFbqdvOAKIdvUodINwN_8z9zbkifQFmp3JJzk4qDqwodNi75jLgYNLEf8tkwPj--5joy0)

First of all lets configure the RMW with the required static memory. The `app-colcon.meta` should looks like:

```
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=2",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=2",
                "-DRMW_UXRCE_MAX_SERVICES=0",
                "-DRMW_UXRCE_MAX_CLIENTS=0",
                "-DRMW_UXRCE_MAX_HISTORY=4",
            ]
        }
    }
}
```

On the other hand, `app.c` should looks like:

```c
#include <allocators.h>

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include "rosidl_generator_c/string_functions.h"
#include <std_msgs/msg/header.h>

#include <rmw_uros/options.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

// FreeRTOS thread for triggering a publication guard condition
void * trigger_guard_condition(void *args){
  rcl_guard_condition_t * guard_condition = (rcl_guard_condition_t *)args;

  while(true){
    rcl_trigger_guard_condition(guard_condition);
    sleep(5);
  }
}

// App main function
void appMain(void *argument)
{
  
  //Init RCL options
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&options, rcl_get_default_allocator());

  // Set Micro-XRCE-DDS client key
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&options);
  RCCHECK(rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options))
  
  // Init RCL context
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init(0, NULL, &options, &context);

  // Create a node
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_init(&node, "pingpong", "", &context, &node_ops);

  // Create a ping ping_publisher
  rcl_publisher_options_t ping_publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t ping_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/micro-ROS/ping", &ping_publisher_ops);

  // Create a ping pong_publisher
  rcl_publisher_options_t pong_publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t pong_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/micro-ROS/pong", &pong_publisher_ops);

  // Create a pong subscriber
  rcl_subscription_options_t pong_subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t pong_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&pong_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/micro-ROS/pong", &pong_subscription_ops);

  // Create a ping subscriber
  rcl_subscription_options_t ping_subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t ping_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&ping_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/micro-ROS/ping", &ping_subscription_ops);

  // Create a guard condition
  rcl_guard_condition_t guard_condition = rcl_get_zero_initialized_guard_condition();
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_guard_condition_init(&guard_condition, &context, guard_condition_options);
  
  // Create a thread that triggers the guard condition
  pthread_t guard_condition_thread;
  pthread_create(&guard_condition_thread, NULL, trigger_guard_condition, &guard_condition);

  // Create a wait set
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_wait_set_init(&wait_set, 2, 1, 0, 0, 1, 0, &context, rcl_get_default_allocator());

  // Create and allocate the pingpong message
  std_msgs__msg__Header msg;
  msg.seq = rand();
  msg.frame_id.capacity = 0;

  // frame_id should contain node identity
  
  int pong_count = 0;
  
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

    // Check if it is time to send a ping
    if (wait_set.guard_conditions[index_guardcondition]) {
      msg.seq = rand();
      pong_count = 0;
      rcl_publish(&ping_publisher, (const void*)&msg, NULL);
      printf("Ping send seq 0x%x\n", msg.seq);
    }
    
    // Check if some pong is received
    if (wait_set.subscriptions[index_pong_subscription]) {
      std_msgs__msg__Header rcv_msg;
      rc = rcl_take(wait_set.subscriptions[index_subscription], &rcv_msg, NULL, NULL);
      if(rcv_msg.seq == msg.seq) {
          pong_count++;
          printf("Pong for seq 0x%x (%d)\n", msg.seq, pong_count);
      }
    }

    // Check if some ping is received and answer it
    if (wait_set.subscriptions[index_ping_subscription]) {
      std_msgs__msg__Header rcv_msg;
      rc = rcl_take(wait_set.subscriptions[index_subscription], &rcv_msg, NULL, NULL);

      // Dont pong my own pings
      if(rcv_msg.seq != msq.seq){
        printf("Ping received with seq 0x%x (%d). Answering.\n", rcv_msg.seq);
        rcl_publish(&pong_publisher, (const void*)&rcv_msg, NULL);
      }
    }
    
    usleep(10000);
  } while (true);
}
```

<!-- http://www.plantuml.com/plantuml/uml/bOwnIWGn48RxFCNFzSkoUG2vqce5jHEHi1dtWZkPa6GByNntavZY40yqnMJu-ORlFwPiOjvvK-dD-M2YOR1uMKvHc93ZJafvoMML07d5h1NAE-DHWblg_nu8vnwEx9Oem_tTmnYSNqkIidtXjARnbeob-2ifLqZo5jMjQWLBU9hBd9u1ap1F-5308bpEoMSSVhWFG0FhFbqdvOAKIdvUodINwN_8z9zbkifQFmp3JJzk4qDqwodNi75jLgYNLEf8tkwPj--5joy0 -->


<!-- [![demo](https://asciinema.org/a/113463.svg)](https://asciinema.org/a/113463?autoplay=1) -->
