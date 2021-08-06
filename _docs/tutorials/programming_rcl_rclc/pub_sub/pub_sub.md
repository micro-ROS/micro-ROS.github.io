---
title: Publishers and Subscriptions
permalink: /docs/tutorials/programming_rcl_rclc/pub_sub/
---

ROS 2 publishers and subscribers are the basic communication mechanism between nodes using topics. Further information about ROS 2 publish–subscribe pattern can be found [here](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html).

Ready to use code related to this tutorial can be found in [`micro-ROS-demos/rclc/int32_publisher`](https://github.com/micro-ROS/micro-ROS-demos/blob/foxy/rclc/int32_publisher/main.c) and [`micro-ROS-demos/rclc/int32_subscriber`](https://github.com/micro-ROS/micro-ROS-demos/blob/foxy/rclc/int32_subscriber/main.c) folders. Fragments of code from this examples are used on this tutorial.

## <a name="pub"/>Publisher

### <a name="pub_init"/>Initialization

Starting from a code where RCL is initialized and a micro-ROS node is created, there are tree ways to initialize a publisher depending on the desired quality-of-service configuration:
  
// TODO: explain and link diferences between each approach on QoS section

- Reliable:
  ```C
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Creates a reliable rcl publisher
  rcl_ret_t rc = rclc_publisher_init_default(&publisher, &node, &type_support, &topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

  // TODO: move to micro-ROS features section?
  Reliable publishers will wait for confirmation for each published message, which leads to blocking `rcl_publish` calls, `rmw-microxrcedds` offers an API to configure the acknowledgement timeout for each publisher:

  ```C
  // Set confirmation timeout in milliseconds
  int ack_timeout = 1000; 
  rc = rmw_uros_set_publisher_session_timeout(&publisher, ack_timeout);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```
  
  The default value for all publishers is configured at compilation time by the cmake variable `RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT`.

- Best effort:
  ```C
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Creates a best effort rcl publisher
  rcl_ret_t rc = rclc_publisher_init_best_effort(&publisher, &node, &type_support, &topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Custom QoS:

  ```C
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Set publisher QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

  // Creates a rcl publisher with customized quality-of-service options
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

Note: `rcl_publish` is thread safe and can be called from multiple threads.
  
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

  // Initialize a realiable subscriber
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

  // Initialize a subscriber with customized quality-of-service options
  rcl_ret_t rc = rclc_subscription_init(&subscriber, &node, &type_support, &topic_name, qos_profile);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

### <a name="sub_callback"/>Callbacks

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
// Message object to save publication data
std_msgs__msg__Int32 msg;

// Add subscription to the executor
rcl_ret_t rc = rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

if (RCL_RET_OK != rc) {
  ...  // Handle error
  return -1;
}
```

### <a name="pubsub_end"/>Cleaning Up

To destroy an initialized publisher or subscriber:

```C
// Destroy publisher and subscriber
rcl_publisher_fini(&publisher, &node);
rcl_subscription_fini(&subscriber, &node);
```

After finishing the publisher/subscriber, the node will no longer be advertising that it is publishing/listening on the topic.

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.