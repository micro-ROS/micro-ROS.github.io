---
title: micro-ROS utilities
permalink: /docs/tutorials/programming_rcl_rclc/micro-ROS/
---

// TODO: Change section name

- [Time sync](#time-sync)
- [Ping agent](#ping-agent)
- [Continous serialization](#continous-serialization)

## Time sync
micro-ROS Clients can synchronize their epoch time with the connected Agent, this can be very useful when working in embedded environments that do not provide any time synchronization mechanism. 
This utility is based on the NTP protocol, taking into account delays caused by the transport layer. An usage example can be found on [`micro-ROS-demos/rclc/epoch_synchronization`](https://github.com/micro-ROS/micro-ROS-demos/blob/galactic/rclc/epoch_synchronization/main.c).

```c
// Sync timeout
const int timeout_ms = 1000;

// Synchronize time with the agent
rmw_uros_sync_session(timeout_ms);

if (rmw_uros_epoch_synchronized()) 
{
    // Get time in milliseconds or nanoseconds
    int64_t time_ms = rmw_uros_epoch_millis();
    int64_t time_ns = rmw_uros_epoch_nanos();
}
```
  
## Ping agent
The Client can test the connection with the Agent with the ping utility. This functionality can be used even when the micro-ROS context has not yet been initialized, which is useful to test the connection before trying to connect to the Agent. An example can be found on [`micro-ROS-demos/rclc/ping_uros_agent`](https://github.com/micro-ROS/micro-ROS-demos/blob/galactic/rclc/ping_uros_agent/main.c).

```c
// Timeout for each attempt
const int timeout_ms = 1000;

// Number of attemps
const uint8_t attemps = 5;

// Ping the agent
rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms, attempts);

if (RMW_RET_OK == ping_result) 
{
    // micro-ROS Agent is reachable
    ...
} 
else 
{
    // micro-ROS Agent is not available
    ...
}
```

*Note: `rmw_uros_ping_agent` is thread safe.*

## Continous serialization

-```c
void rmw_uros_set_continous_serialization_callbacks(
rmw_publisher_t * publisher,
rmw_uros_continous_serialization_size size_cb,
rmw_uros_continous_serialization serialization_cb);
```