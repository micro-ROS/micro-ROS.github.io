---
title: Publishers and Subscriptions
permalink: /docs/tutorials/programming_rcl_rclc/pub_sub/
---

## <a name="pub"/>Publisher

### <a name="pub_init"/>Initialization

Starting from a code where RCL is initialized and a micro-ROS node is created, there are tree ways to initialize a publisher:
  
// TODO: explain and link diferences between each approach on QoS section

- Reliable publisher:
  ```C
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // TODO: explain type_support?
  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Create a reliable rcl publisher.
  rcl_ret_t rc = rclc_publisher_init_default(&publisher, &node, &type_support, &topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```
  
  Reliable publishers will wait for confirmation for each published message, which leads to blocking `rcl_publish` calls. The `rmw-microxrcedds` offer an API to configure the acknowledgement timeout for each publisher:

  ```C
  // Set confirmation timeout in milliseconds
  int publish_timeout = 1000; 
  rc = rmw_uros_set_publisher_session_timeout(&publisher, publish_timeout);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```
  
  The default value for all publishers is configured at compilation time by the cmake variable `RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT`.

- Best effort publisher:

  // TODO: explain in QoS section?
  Publish the message without reception confirmation, allowing a faster publish rate.

  ```C
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Creates an rcl publisher with quality-of-service option best effort
  rcl_ret_t rc = rclc_publisher_init_best_effort(&publisher, &node, &type_support, &topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Add QoS API

  ```C
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Set publisher QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

  // Creates an rcl publisher with customized quality-of-service options
  rcl_ret_t rc = rclc_publisher_init(&publisher, &node, &type_support, &topic_name, qos_profile);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

## <a name="pub_publish"/>Publish a message

// TODO: explain message memory allocation and link to tutorial
// TODO: explain periodic publication and link to timers
```C
// Int32 message object
std_msgs__msg__Int32 msg; 

// Set message value
msg.data = 0;

// Publish message
rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}
```

Note: `rcl_publish` is thread safe and can be called from multiple threads
  
## <a name="pub_end"/>Destroy a publisher

Deallocates memory
After calling, the node will no longer be advertising that it is publishing
on this topic (assuming this is the only publisher on this topic).

```C
// Destroy publisher
rcl_ret_t rc = rcl_publisher_fini(&publisher, &node);

if (rc == RCL_RET_OK) {
  ...  // Handle error
  return -1;
}
```

Note: Publishers needs to be destroyed before its containing node

## <a name="sub"/>Subscription

### <a name="sub_init"/>Initialization

The suscriptor initialization is almost identical to the publisher one:

- Reliable (default):
  ```C
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Set client QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

  // Initialize subscriber with default configuration
  rcl_ret_t rc = rclc_subscription_init_default(&subscriber, &node, &type_support, &topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Best effort:

  ```C
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Initialize best effort subscriber
  rcl_ret_t rc = rclc_subscription_init_best_effort(&subscriber, &node, &type_support, &topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Add QoS API

  ```C
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Set client QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

  // Initialize server with customized quality-of-service options
  rcl_ret_t rc = rclc_subscription_init(&subscriber, &node, &type_support, &topic_name, qos_profile);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

### <a name="sub_callback"/>Callbacks

// TODO: explain message memory allocation and link to tutorial

```C
void subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Process message
  printf("Received: %d\n", msg->data);
}
```

Once the subscriber and the executor are initialized, the subscriber callback must be added to the executor to receive incoming publications once the executor is spinning:

```C
// Message object
std_msgs__msg__Int32 msg;

// Add subscription to the executor
rcl_ret_t rc = rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

if (RCL_RET_OK != rc) {
  ...  // Handle error
  return -1;
}
```

### <a name="sub_end"/>Destroy a subscriber
Destroy subscriber (Publisher needs to be destroyed manually before the node)
Destroys any automatically created infrastructure and deallocates memory.

```C
// Destroy
rcl_ret_t rc = rcl_publisher_fini(&subscriber, &node);

if (rc == RCL_RET_OK) {
  printf("Published message with value: %d\n", msg.data);
}
```
