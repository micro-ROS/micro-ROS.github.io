---
title: micro-ROS utilities
permalink: /docs/tutorials/programming_rcl_rclc/micro-ROS/
---

## <a name="sub"/>micro-ROS features


- Custom transport  
rmw_ret_t rmw_uros_set_custom_transport(
  bool framing,
  void * args,
  open_custom_func open_cb,
  close_custom_func close_cb,
  write_custom_func write_cb,
  read_custom_func read_cb);


- Time sync

```C

bool rmw_uros_epoch_synchronized();
int64_t rmw_uros_epoch_millis();
int64_t rmw_uros_epoch_nanos();
rmw_ret_t rmw_uros_sync_session(const int timeout_ms);

```

- Ping agent

rmw_ret_t rmw_uros_ping_agent(const int timeout_ms, const uint8_t attempts);


- Init options ??

- Discovery ??

- Continous serialization ??
void rmw_uros_set_continous_serialization_callbacks(
rmw_publisher_t * publisher,
rmw_uros_continous_serialization_size size_cb,
rmw_uros_continous_serialization serialization_cb);


// Add publisher / service / client timeout here?