---
title: Quality of service
permalink: /docs/tutorials/programming_rcl_rclc/qos/
---

<img src="https://img.shields.io/badge/Written_for-Humble-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Rolling-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Iron-green" style="display:inline"/>

- [Reliable QoS](#reliable-qos)
- [Best Effort](#best-effort)
- [Custom QoS configuration](#custom-qos-configuration)

## Reliable QoS

Reliable communication implies a confirmation for each message sent. This mode can detect errors in the communication process at the cost of increasing the message latency and the resources usage.

This message confirmation proccess can increase blocking time on `rcl_publish` or executor spin calls as reliable publishers, services and clients will wait for acknowledgement for each sent message. `rmw-microxrcedds` offers an API to individually configure the acknowledgement timeout on them:

  ```c
  // Confirmation timeout in milliseconds
  int ack_timeout = 1000;

  // Set reliable publisher timeout
  rc = rmw_uros_set_publisher_session_timeout(&publisher, ack_timeout);

  // Set reliable service server timeout
  rc = rmw_uros_set_service_session_timeout(&service, ack_timeout);

  // Set reliable service client timeout
  rc = rmw_uros_set_client_session_timeout(&client, ack_timeout);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

  The default value for all publishers is configured at compilation time by the cmake variable `RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT`.

## Best Effort

In best effort mode no acknowledgement is needed, the messages sent are expected to be received. This method improves publication throughput and reduces resources usage but is vulnerable to communication errors.

## Custom QoS configuration

The user can customize their own QoS using the available `rmw_qos_profile_t` struct:

```c
/// ROS MiddleWare quality of service profile.
typedef struct RMW_PUBLIC_TYPE rmw_qos_profile_t
{
  enum rmw_qos_history_policy_t history;
  /// Size of the message queue.
  size_t depth;
  /// Reliabiilty QoS policy setting
  enum rmw_qos_reliability_policy_t reliability;
  /// Durability QoS policy setting
  enum rmw_qos_durability_policy_t durability;
  /// The period at which messages are expected to be sent/received
  struct rmw_time_t deadline;
  /// The age at which messages are considered expired and no longer valid
  struct rmw_time_t lifespan;
  /// Liveliness QoS policy setting
  enum rmw_qos_liveliness_policy_t liveliness;
  /// The time within which the RMW node or publisher must show that it is alive
  struct rmw_time_t liveliness_lease_duration;

  /// If true, any ROS specific namespacing conventions will be circumvented.
  bool avoid_ros_namespace_conventions;
} rmw_qos_profile_t;
```
