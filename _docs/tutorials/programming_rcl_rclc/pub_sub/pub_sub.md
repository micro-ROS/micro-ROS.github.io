---
title: Publishers and subscribers
permalink: /docs/tutorials/programming_rcl_rclc/pub_sub/
---

<img src="https://img.shields.io/badge/Written_for-Humble-green" style="display:inline"/>  <img src="https://img.shields.io/badge/Tested_on-Rolling-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Iron-green" style="display:inline"/>

ROS 2 publishers and subscribers are the basic communication mechanism between nodes using topics. Further information about ROS 2 publish–subscribe pattern can be found [here](https://docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html).

Ready to use code related to this concepts can be found in [`micro-ROS-demos/rclc/int32_publisher`](https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/int32_publisher/main.c) and [`micro-ROS-demos/rclc/int32_subscriber`](https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/int32_subscriber/main.c) folders. Fragments of code from this examples are used on this tutorial.

- [Publisher](#publisher)
  - [Initialization](#initialization)
  - [Publish a message](#publish-a-message)
- [Subscription](#subscription)
  - [Initialization](#initialization-1)
  - [Callbacks](#callbacks)
- [Message initialization](#message-initialization)
- [Cleaning Up](#cleaning-up)

## Publisher

### Initialization

Starting from a code where RCL is initialized and a micro-ROS node is created, there are three ways to initialize a publisher depending on the desired quality-of-service configuration:

- Reliable (default):
  ```c
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Creates a reliable rcl publisher
  rcl_ret_t rc = rclc_publisher_init_default(
    &publisher, &node,
    type_support, topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Best effort:
  ```c
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Creates a best effort rcl publisher
  rcl_ret_t rc = rclc_publisher_init_best_effort(
    &publisher, &node,
    type_support, topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Custom QoS:

  ```c
  // Publisher object
  rcl_publisher_t publisher;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Set publisher QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

  // Creates a rcl publisher with customized quality-of-service options
  rcl_ret_t rc = rclc_publisher_init(
    &publisher, &node,
    type_support, topic_name, qos_profile);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

  For a detail on the available QoS options and the advantages and disadvantages between reliable and best effort modes, check the [QoS tutorial](../qos/).

### Publish a message

To publish messages to the topic:

```c
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

For periodic publications,  `rcl_publish` can be placed inside a timer callback. Check the [Executor and timers](../executor/) section for details.

Note: `rcl_publish` is thread safe and can be called from multiple threads.

## Subscription

### Initialization

The suscriptor initialization is almost identical to the publisher one:

- Reliable (default):
  ```c
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Initialize a reliable subscriber
  rcl_ret_t rc = rclc_subscription_init_default(
    &subscriber, &node,
    type_support, topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Best effort:

  ```c
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Initialize best effort subscriber
  rcl_ret_t rc = rclc_subscription_init_best_effort(
    &subscriber, &node,
    type_support, topic_name);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

- Custom QoS:

  ```c
  // Subscription object
  rcl_subscription_t subscriber;
  const char * topic_name = "test_topic";

  // Get message type support
  const rosidl_message_type_support_t * type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

  // Set client QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_default;

  // Initialize a subscriber with customized quality-of-service options
  rcl_ret_t rc = rclc_subscription_init(
    &subscriber, &node,
    type_support, topic_name, qos_profile);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

For a detail on the available QoS options and the advantages and disadvantages between reliable and best effort modes, check the [QoS tutorial](../qos/).

### Callbacks
The executor is responsible to call the configured callback when a message is published.
The function will have the message as its only argument, containing the values sent by the publisher:

```c
// Function prototype:
void (* rclc_subscription_callback_t)(const void *);

// Implementation example:
void subscription_callback(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Process message
  printf("Received: %d\n", msg->data);
}
```

Once the subscriber and the executor are initialized, the subscriber callback must be added to the executor to receive incoming publications once its spinning:

```c
// Message object to receive publisher data
std_msgs__msg__Int32 msg;

// Add subscription to the executor
rcl_ret_t rc = rclc_executor_add_subscription(
  &executor, &subscriber, &msg,
  &subscription_callback, ON_NEW_DATA);

if (RCL_RET_OK != rc) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive messages
rclc_executor_spin(&executor);
```

## Message initialization
Before publishing or receiving a message, it may be necessary to initialize its memory for types with strings or sequences.
Check the [Handling messages memory in micro-ROS](../../advanced/handling_type_memory/) section for details.

## Cleaning Up

After finishing the publisher/subscriber, the node will no longer be advertising that it is publishing/listening on the topic.
To destroy an initialized publisher or subscriber:

```c
// Destroy publisher
rcl_publisher_fini(&publisher, &node);

// Destroy subscriber
rcl_subscription_fini(&subscriber, &node);
```

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.
