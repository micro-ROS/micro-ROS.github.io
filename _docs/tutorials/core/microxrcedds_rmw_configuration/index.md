---
title: RMW & Micro XRCE-DDS Configuration
permalink: /docs/tutorials/core/microxrcedds_rmw_configuration/
---

Since micro-ROS is intended to work on extremely low resources systems, its middleware layer is highly configurable. This configurability, along with the fact that the middleware layer does not use dynamic memory, allows the users to determine how much static memory is going to be allocated.

If you want to read more about the middleware layer follow this link: [Micro XRCE-DDS](https://micro-xrce-dds.readthedocs.io/en/latest/)

The ROS 2 middleware layer for Micro XRCE-DDS or rmw_microxrcedds allows two types of configuration:
- Compile-time configuration.
- Run-time configuration.

Ready to use code related to this tutorial can be found in `micro-ROS-demos/rcl/configuration_example/` folder in [`micro-ROS-demos` repo](https://github.com/micro-ROS/micro-ROS-demos/tree/dashing/rcl/configuration_example).

## Compile-time configuration

The following parameters can be configured through CMake arguments:

<!-- TODO: Related errors (FAQ) -->

| Parameter name | Description |  Default value |
| - | - | - |
| RMW_UXRCE_TRANSPORT | Sets Micro XRCE-DDS transport to use: udp, serial or custom. | udp |
| RMW_UXRCE_CREATION_MODE | Sets creation mode in Micro XRCE-DDS: xml or refs. | xml |
| RMW_UXRCE_MAX_HISTORY | This value sets the number of MTUs to buffer. Micro XRCE-DDS client configuration provides its size. | 4 |
| RMW_UXRCE_MAX_NODES | This value sets the maximum number of nodes. | 4 |
| RMW_UXRCE_MAX_PUBLISHERS | This value sets the maximum number of publishers available. | 4 |
| RMW_UXRCE_MAX_SUBSCRIPTIONS | This value sets the maximum number of subscriptions available. | 4 |
| RMW_UXRCE_MAX_SERVICES | This value sets the maximum number of services available. | 4 |
| RMW_UXRCE_MAX_CLIENTS | This value sets the maximum number of clients available. | 4 |
| RMW_UXRCE_NODE_NAME_MAX_LENGTH | This value sets the maximum number of characters for a node name. | 128 |
| RMW_UXRCE_TOPIC_NAME_MAX_LENGTH | This value sets the maximum number of characters for a topic name. | 100 |
| RMW_UXRCE_TYPE_NAME_MAX_LENGTH | This value sets the maximum number of characters for a type name. | 128 |
| RMW_UXRCE_XML_BUFFER_LENGTH | This value sets the maximum number of characters for a XML buffer used internally. | 600 |
| RMW_UXRCE_REF_BUFFER_LENGTH | This value sets the maximum number of characters for a reference buffer used internally. | 100 |
| RMW_UXRCE_DEFAULT_SERIAL_DEVICE | Sets the agent default serial port. | /dev/ttyAMA0 |
| RMW_UXRCE_DEFAULT_UDP_IP | Sets the agent default IP address. | 127.0.0.1 |
| RMW_UXRCE_DEFAULT_UDP_PORT | Sets the agent default IP port. | 8888 |

The micro-ROS way to pass CMake arguments to the build system is using `colcon.meta` file. For example, increasing the number of statically allocated publishers looks like that:

```
{ 
  "names": {
    "rmw_microxrcedds": { 
      "cmake-args": [ 
        "-DRMW_UXRCE_MAX_PUBLISHERS=6" 
      ] 
    }
  }
}
```

## Run-time configuration

Although there are some build time parameters related to client to agent connection (such as **CONFIG_RMW_DEFAULT_UDP_PORT**, **CONFIG_RMW_DEFAULT_UDP_IP** and **CONFIG_RMW_DEFAULT_SERIAL_DEVICE**) these kinds of parameters can also be configured at run-time. The following example code shows the API calls needed to set the agent's IP address, port or serial device:

```c 
#include <rmw_uros/options.h>

// Init RCL options and context
rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
rcl_context_t context = rcl_get_zero_initialized_context();
rcl_init_options_init(&init_options, rcl_get_default_allocator());

// Take RMW options from RCL options
rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

// TCP/UDP case: Set RMW IP parameters
rmw_uros_options_set_udp_address("127.0.0.1", "8888", rmw_options);

// Serial case: Set RMW serial device parameters
mw_uros_options_set_serial_device("/dev/ttyAMA0", rmw_options)

// Set RMW client key
rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options);

// Init RCL
rcl_init(0, NULL, &init_options, &context);

// ... micro-ROS code ...
```

Notice that is also possible to set the Micro XRCE-DDS `client_key`, which would otherwise be set randomly. This feature is useful for reusing DDS entities already created on the agent side. Further information can be found [here](https://micro-xrce-dds.readthedocs.io/en/latest/deployment.html#configurate-the-publisher).
