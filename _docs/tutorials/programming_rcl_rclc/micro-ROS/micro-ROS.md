---
title: micro-ROS utilities
permalink: /docs/tutorials/programming_rcl_rclc/micro-ROS/
---

// TODO

- [Time sync](#time-sync)
- [Ping agent](#ping-agent)

## Time sync

```c
bool rmw_uros_epoch_synchronized();
int64_t rmw_uros_epoch_millis();
int64_t rmw_uros_epoch_nanos();
rmw_ret_t rmw_uros_sync_session(const int timeout_ms);
```

## Ping agent

```c
rmw_ret_t rmw_uros_ping_agent(const int timeout_ms, const uint8_t attempts);
```

- Init options ??

- Discovery ??

- Continous serialization ??
-```c
void rmw_uros_set_continous_serialization_callbacks(
rmw_publisher_t * publisher,
rmw_uros_continous_serialization_size size_cb,
rmw_uros_continous_serialization serialization_cb);
```