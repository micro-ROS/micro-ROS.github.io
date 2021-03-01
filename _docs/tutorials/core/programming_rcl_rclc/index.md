---
title: Programming with rcl and rclc
permalink: /docs/tutorials/core/programming_rcl_rclc/
---

In this tutorial, you'll learn the basics of the micro-ROS C API. The major concepts (publishers, subscriptions, services,timers, ...) are identical with ROS 2. They even rely on the *same* implementation, as the micro-ROS C API is based on the ROS 2 client support library (rcl), enriched with a set of convenience functions by the package [rclc](https://github.com/ros2/rclc/). That is, rclc does not add a new layer of types on top of rcl (like rclcpp and rclpy do) but only provides functions that ease the programming with the rcl types. New types are introduced only for concepts that are missing in rcl, such as the concept of an executor.

* [Creating a node](#node)
* [Publishers and subscriptions](#pub_sub)
* [Services](#services)
* [Timers](#timers)
* [Lifecycle](#lifecycle)
* [Rclc Executor](#rclc_executor)

## <a name="node"/>Creating a Node

To simplify the creation of a node with rcl, rclc provides two functions `rclc_support_init(..)` and `rclc_node_init_default(..)` in [rclc/init.h](https://github.com/ros2/rclc/blob/master/rclc/include/rclc/init.h) and [rclc/node.h](https://github.com/ros2/rclc/blob/master/rclc/include/rclc/node.h), respectively. The first lines of the main function of a micro-ROS programm are:

```C
rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;
rcl_ret_t rc;

rc = rclc_support_init(&support, argc, argv, &allocator);
if (rc != RCL_RET_OK) {
  ...  // Some error reporting.
  return -1;
}

rcl_node_t my_node = rcl_get_zero_initialized_node();
rc = rclc_node_init_default(&my_node, "my_node_name", "my_namespace", &support);
if (rc != RCL_RET_OK) {
  ...  // Some error reporting.
  return -1;
}
```

## <a name="pub_sub"/>Publishers and Subscriptions

Publishers and subscribers are most easily created with the rclc package.

Creating a publisher by `rclc_publisher_init_default(..)` from [rclc/publisher.h](https://github.com/ros2/rclc/blob/master/rclc/include/rclc/publisher.h):

```C
rcl_publisher_t my_pub;
std_msgs__msg__String my_msg;
const char * my_topic = "topic_0";
const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

rc = rclc_publisher_init_default(&my_pub, &my_node, &my_type_support, &my_topic_name);
if (RCL_RET_OK != rc) {
  printf("Error in rclc_publisher_init_default.\n");
  return -1;
}
```

Initializing a message:

```C
std_msgs__msg__String__init(&pub_msg);
const unsigned int PUB_MSG_SIZE = 20;
char pub_string[PUB_MSG_SIZE];
snprintf(pub_string, 13, "%s", "Hello World!");
rosidl_generator_c__String__assignn(&pub_msg, pub_string, PUB_MSG_SIZE);
```

Creating a subscription by `rclc_subscription_init_default(..)` from [rclc/subscription.h](https://github.com/ros2/rclc/blob/master/rclc/include/rclc/subscription.h):

```C
rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
rc = rclc_subscription_init_default(&my_sub, &my_node, &my_type_support, &my_topic_name);
if (rc != RCL_RET_OK) {
  printf("Failed to create subscriber.\n");
  return -1;
}
```

## <a name="services"/>Services

ROS 2 services is another communication mechanism between nodes. Services implement a client-server paradigm based on ROS 2 messages and types. Further information about ROS 2 services can be found [here](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)

Ready to use code related to this tutorial can be found in [`micro-ROS-demos/rclc/addtwoints_server`](https://github.com/micro-ROS/micro-ROS-demos/blob/foxy/rclc/addtwoints_server/main.c) and [`micro-ROS-demos/rclc/addtwoints_client`](https://github.com/micro-ROS/micro-ROS-demos/blob/foxy/rclc/addtwoints_client/main.c) folders.

Note: Services are not supported in rclc package yet. Therefore, for the moment, the configuration is described using the RCL layer. However, we are working to port them to the RCLC soon.

Starting from a code where RCL is initialized and a micro-ROS node is created, these steps are required in order to generate a service server:

```C
// Creating service server and options
rcl_service_options_t service_options = rcl_service_get_default_options();
rcl_service_t server = rcl_get_zero_initialized_service();

// Initializing service server
rcl_service_init(&server, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "addtwoints", &service_options);

// Init service server wait set
rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
rcl_wait_set_init(&wait_set, 0, 0, 0, 0, 1, 0, &context, rcl_get_default_allocator());

```

On the other hand the service client initialization looks like that:

```C
// Creating service client and options
rcl_client_options_t client_options = rcl_client_get_default_options();
rcl_client_t client = rcl_get_zero_initialized_client();

// Initializing service client
rcl_client_init(&client, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "addtwoints", &client_options)

// Init service client wait set
rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
rcl_wait_set_init(&wait_set, 0, 0, 0, 1, 0, 0, &context, rcl_get_default_allocator());
```

First of all, by looking at `AddTwoInts.srv` type definition it is possible to determine request and reply elements of the service. Service client will make a request with two integers and service server should send its sum as a response.

```C
int64 a
int64 b
---
int64 sum
```

Once service client and server are configured, service client can perform a request and wait for reply:

```C
// Creating a service request
int64_t seq;
example_interfaces__srv__AddTwoInts_Request req;
req.a = 24;
req.b = 42;

// Sending the request
rcl_send_request(&client, &req, &seq)
printf("Send service request %d + %d. Seq %ld\n",(int)req.a, (int)req.b, (int)seq);

// Wait for response
bool done = false;
do {
    rcl_wait_set_clear(&wait_set);

    size_t index;
    rcl_wait_set_add_client(&wait_set, &client, &index);

    rcl_wait(&wait_set, RCL_MS_TO_NS(1));

    // If wait set client element is not null, response is ready
    if (wait_set.clients[index]) {
        rmw_request_id_t req_id;

        // Create a service response struct
        example_interfaces__srv__AddTwoInts_Response res;

        // Take the response
        rcl_ret_t rc = rcl_take_response(&client, &req_id, &res);

        if (RCL_RET_OK == rc) {
            printf("Received service response %d + %d = %d. Seq %d\n",(int)req.a, (int)req.b, (int)res.sum,req_id.sequence_number);
            done = true;
        }
    }
} while ( !done );
```

On service server side, the ROS 2 node should be waiting for service requests and generate service replies:

```C
while(1){
    rcl_wait_set_clear(&wait_set);

    size_t index;
    rcl_wait_set_add_service(&wait_set, &service, &index);

    rcl_wait(&wait_set, RCL_MS_TO_NS(1));

    // If wait set service element is not null, request is ready
    if (wait_set.services[index]) {
        rmw_request_id_t req_id;

        // Create a service request struct
        example_interfaces__srv__AddTwoInts_Request req;

        // Take the request
        rcl_take_request(&service, &req_id, &req);

        printf("Service request value: %d + %d. Seq %d\n", (int)req.a, (int)req.b, (int)req_id.sequence_number);

        // Create a service response, fill the result and send it
        example_interfaces__srv__AddTwoInts_Response res;
        res.sum = req.a + req.b;
        rcl_send_response(&service, &req_id,&res);
    }
}
```

## <a name="timers"/>Timers

A timer can be created with the rclc-package with the function
`rclc_timer_init_default(..)` in [rclc/timer.h](https://github.com/ros2/rclc/blob/master/rclc/include/rclc/timer.h):

```C
// create a timer, which will call the publisher with period=`timer_timeout` ms in the 'my_timer_callback'
rcl_timer_t my_timer = rcl_get_zero_initialized_timer();
const unsigned int timer_timeout = 1000;
rc = rclc_timer_init_default(&my_timer, &support, RCL_MS_TO_NS(timer_timeout), my_timer_callback);
if (rc != RCL_RET_OK) {
  printf("Error in rcl_timer_init_default.\n");
  return -1;
} else {
  printf("Created timer with timeout %d ms.\n", timer_timeout);
}
```

## <a name="lifecycle"/>Lifecycle

The rclc lifecycle package provides convenience functions in C to bundle an rcl node with the ROS 2 Node Lifecycle state machine, similar to the [rclcpp Lifecycle Node](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp) for C++.

This tutorial show-cases how to set up an rclc lifecycle node, transition through its lifecycle states, and assign callbacks to lifecycle transitions.

### Initialization

Creation of a lifecycle node as a bundle of an rcl node and the rcl lifecycle state machine.

```C
#include "rclc_lifecycle/rclc_lifecycle.h"

rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;
rcl_ret_t rc;

// create rcl node
rc = rclc_support_init(&support, argc, argv, &allocator);
rcl_node_t my_node = rcl_get_zero_initialized_node();
rc = rclc_node_init_default(&my_node, "my_lifecycle_node", "rclc", &support);

// rcl state machine
rcl_lifecycle_state_machine_t state_machine =
  rcl_lifecycle_get_zero_initialized_state_machine();
...

// create the lifecycle node
rclc_lifecycle_node_t my_lifecycle_node;
rcl_ret_t rc = rclc_make_node_a_lifecycle_node(
  &my_lifecycle_node,
  &my_node,
  &state_machine,
  &allocator);
```

Optionally create hooks for lifecycle state changes.

```C
// declare callback
rcl_ret_t my_on_configure() {
  printf("  >>> my_lifecycle_node: on_configure() callback called.\n");
  return RCL_RET_OK;
}
...

// register callbacks
rclc_lifecycle_register_on_configure(&my_lifecycle_node, &my_on_configure);
```

### Running

Change states of the lifecycle node, e.g.

```C
bool publish_transition = true;
rc += rclc_lifecycle_change_state(
  &my_lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE,
  publish_transition);
rc += rclc_lifecycle_change_state(
  &my_lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE,
  publish_transition);
...
```

Except for error processing transitions, transitions are usually triggered from outside, e.g., by ROS 2 services.

### Cleaning Up

To clean everything up, simply do

```C
rc += rcl_lifecycle_node_fini(&my_lifecycle_node, &allocator);
```

### Example and Limitations

An example of the rclc Lifecycle Node is given in the file `lifecycle_node.c` in the [rclc_examples](https://github.com/ros2/rclc/tree/master/rclc_examples) package.

The state machine publishes state changes, however, lifecycle services are not yet exposed via ROS 2 services ([ros2/rclc#40](https://github.com/ros2/rclc/issues/40)).

## <a name="rclc_executor"/>rclc Executor

The rclc Executor provides a C API to manage the execution of subscription and timer callbacks, similar to the [rclcpp Executor](https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/executor.hpp) for C++. The rclc Executor is optimized for resource-constrained devices and provides additional features that allow the manual implementation of deterministic schedules with bounded end-to-end latencies.

In this tutorial we provide two examples:

* Example 1: Hello-World example consisting of one executor and one publisher, timer and subscription.
* Example 2: Triggered execution example, demonstrating the capability of synchronizing the execution of callbacks based on the availability of new messages

Further examples for using the rclc Executor in mobile robotics scenarios and real-time embedded applications can be found in the [rclc](https://github.com/ros2/rclc/tree/master/rclc) repository.

### Example 1: 'Hello World'

To start with, we provide a very simple example for an rclc Executor with one timer and one subscription, so to say, a 'Hello world' example. It consists of a publisher, sending a 'hello world' message to a subscriber, which then prints out the received message on the console.

First, you include some header files, in particular the [rclc/rclc.h](https://github.com/ros2/rclc/blob/master/rclc/include/rclc/rclc.h) and [rclc/executor.h](https://github.com/ros2/rclc/blob/master/rclc/include/rclc/executor.h).

```C
#include <stdio.h>
#include <std_msgs/msg/string.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
```

We define a publisher and two strings, which will be used later.

```C
rcl_publisher_t my_pub;
std_msgs__msg__String pub_msg;
std_msgs__msg__String sub_msg;
```

The subscription callback casts the message parameter `msgin` to an equivalent type of `std_msgs::msg::String` in C and prints out the received message.

```C
void my_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback: I heard: %s\n", msg->data.data);
  }
}
```

The timer callback publishes the message `pub_msg` with the publisher `my_pub`, which is initialized later in `main()`.

```C
void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %s\n", pub_msg.data.data);
    } else {
      printf("Error in timer_callback: Message %s could not be published\n", pub_msg.data.data);
    }
  } else {
    printf("Error in timer_callback: timer parameter is NULL\n");
  }
}
```

After defining the callback functions, we present now the `main()` function. First, some initialization is necessary to create later rcl objects. That is an `allocator` for dynamic memory allocation, and a `support` object, which contains some rcl-objects simplifying the initialization of an rcl-node, an rcl-subscription, an rcl-timer and an rclc-executor.

```C
int main(int argc, const char * argv[])
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }
```

Next, you define a ROS 2 node `my_node` with `rcl_get_zero_initialized_node()` and initialize it with `rclc_executor_init_default()`:

```C
  // create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&my_node, "node_0", "executor_examples", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }
```

You can create a publisher to publish topic 'topic_0' with type std_msg::msg::String with the following code:

```C
const char * topic_name = "topic_0";
const rosidl_message_type_support_t * my_type_support =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

rc = rclc_publisher_init_default(&my_pub, &my_node, my_type_support, topic_name);
if (RCL_RET_OK != rc) {
  printf("Error in rclc_publisher_init_default %s.\n", topic_name);
  return -1;
}
```

Note, that variable `my_pub` was defined globally, so it can be used by the timer callback.

You can create a timer `my_timer` with a period of one second, which executes the callback `my_timer_callback` like this:

```C
  rcl_timer_t my_timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 1000; // in ms
  rc = rclc_timer_init_default(&my_timer, &support, RCL_MS_TO_NS(timer_timeout), my_timer_callback);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init_default.\n");
    return -1;
  } else {
    printf("Created timer with timeout %d ms.\n", timer_timeout);
  }
```

The string `Hello World!` can be assigned directly to the message of the publisher `pub_msg.data`. First the publisher message is initialized with `std_msgs__msg__String__init`. Then you need to allocate memory for `pub_msg.data.data`, set the maximum capacity `pub_msg.data.capacity` and set the length of the message `pub_msg.data.size` accordingly. You can assign the content of the message with `snprintf` of `pub_msg.data.data`.

```C
  // assign message to publisher
  std_msgs__msg__String__init(&pub_msg);
  const unsigned int PUB_MSG_CAPACITY = 20;
  pub_msg.data.data = malloc(PUB_MSG_CAPACITY);
  pub_msg.data.capacity = PUB_MSG_CAPACITY;
  snprintf(pub_msg.data.data, pub_msg.data.capacity, "Hello World!");
  pub_msg.data.size = strlen(pub_msg.data.data);
```

A subscription `my_sub`can be defined like this:

```C
  rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(&my_sub, &my_node, my_type_support, topic_name);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }
```

The global message for this subscription `sub_msg` needs to be initialized with:

```C
  std_msgs__msg__String__init(&sub_msg);
```

Now, all preliminary steps are done, and you can define and initialized the rclc executor with:

```C
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
```

In the next step, executor is initialized with the ROS 2 `context`, the number of communication objects `num_handles` and an `allocator`. The number of communication objects defines the total number of timers and subscriptions, the executor shall manage. In this example, the executor will be setup with one timer and one subscription.

```C
  // total number of handles = #subscriptions + #timers
  unsigned int num_handles = 1 + 1;
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
```

Now, you can add a subscription with the function `rclc_c_executor_add_subscription` with the previously defined subscription `my_sub`, its message `sub_msg`and its callback `my_subscriber_callback`:

```C
rc = rclc_executor_add_subscription(&executor, &my_sub, &sub_msg, &my_subscriber_callback, ON_NEW_DATA);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_subscription. \n");
}
```

The option `ON_NEW_DATA` selects the execution semantics of the spin-method. In this example, the callback of the subscription `my_sub`is only called if new data is available.

Note: Another execution semantics is `ALWAYS`, which means, that the subscription callback is always executed when the spin-method of the executor is called. This option might be useful in cases in which the callback shall be executed at a fixed rate irrespective of new data is available or not. If you choose this option, then the callback will be executed with message argument `NULL` if no new data is available. Therefore you need to make sure, that your callback also accepts `NULL` as message argument.

Likewise, you can add the timer `my_timer` with the function `rclc_c_executor_add_timer`:

```C
rclc_executor_add_timer(&executor, &my_timer);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_timer.\n");
}
```

A key feature of the rclc Executor is that the order of these `rclc-executor-add-*`-functions matters. The order in which these functions are called defines the static processing order of the callbacks when the spin-function of the executor is running.

In this example, the timer was added to the executor before the subscription. Therefore, if the timer is ready and also a new message for the subscription is available, then the timer is executed first and after it the subscription. Such a behavior cannot be defined currently with the rclcpp Executor and is useful to implement a deterministic execution semantics.

Finally, you can run the executor with `rclc_executor_spin()`:

```C
  rclc_executor_spin(&executor);
```

This function runs forever without coming back. In this example, however, we want to publish the message only ten times. Therefore we are using the spin-method `rclc_executor_spin_some`, which spins only once and returns. The wait timeout for checking for new messages at the DDS-queue or waiting timers to get ready is configured to be one second.

```C
for (unsigned int i = 0; i < 10; i++) {
  // timeout specified in nanoseconds (here 1s)
  rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
}
```

At the end, you need to free dynamically allocated memory:

```C
  // clean up
  rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_timer_fini(&my_timer);
  rc += rcl_subscription_fini(&my_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);
  std_msgs__msg__String__fini(&pub_msg);
  std_msgs__msg__String__fini(&sub_msg);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
return 0;
} // main
```

This completes the example. The source code can be found in the package rclc-examples [rclc-examples/example_executor_convenience.c](https://github.com/ros2/rclc/blob/master/rclc_examples/src/example_executor_convenience.c).

#### Example 2: Triggered execution

In robotic applications often multiple sensors are used to improve localization precision. These sensors can have different frequencies, for example, a high frequency IMU sensor and a low frequency laser scanner. One way is to trigger execution upon arrival of a laser scan and only then evaluate the most recent data from the aggregated IMU data.

This example demonstrates the additional feature of the rclc executor to trigger the execution of callbacks based on the availability of input data.

We setup one executor with two publishers, one with 100ms and one with 1000ms period. Then we setup one executor for two subscriptions. Their callbacks shall both be executed if the message of the publisher with the lower frequency arrives.

The output of this code example will look like this:

```C
Created timer 'my_string_timer' with timeout 100 ms.
Created 'my_int_timer' with timeout 1000 ms.
Created subscriber topic_0:
Created subscriber topic_1:
Executor_pub: number of DDS handles: 2
Executor_sub: number of DDS handles: 2
Published: Hello World! 0
Published: Hello World! 1
Published: Hello World! 2
Published: Hello World! 3
Published: Hello World! 4
Published: Hello World! 5
Published: Hello World! 6
Published: Hello World! 7
Published: Hello World! 8
Published: Hello World! 9
Published: 0
Callback 1: Hello World! 9  <---
Callback 2: 0               <---
Published: Hello World! 10
Published: Hello World! 11
Published: Hello World! 12
Published: Hello World! 13
Published: Hello World! 14
Published: Hello World! 15
Published: Hello World! 16
Published: Hello World! 17
Published: Hello World! 18
Published: Hello World! 19
Published: 1
Callback 1: Hello World! 19 <---
Callback 2: 1               <---
```

This output shows, that the callbacks are executed, only if both message have received new data. In that case, the latest data of high-frequency topic is used.

You learn in this tutorial

* how to use pre-defined trigger conditions
* how to write custom-defined trigger conditions
* how to run multiple executors
* how to setup quality-of-service parameters for a subscription

We start with the necessary includes for string and int messages, `<std_msgs/msg/string.h>` and `std_msgs/msg/int32.h` respectivly. Then the necessary includes follow for the rclc convenience functions `rclc.h` and the the rclc executor `executor.h`:

```C
#include <stdio.h>
#include <unistd.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

#include <rclc/executor.h>
#include <rclc/rclc.h>
```

Then, global variables for the publishers and subscriptions as well as their messages are defined, which are initialized in the `main()` function and used in the corresponding callbacks:

```C
rcl_publisher_t my_pub;
rcl_publisher_t my_int_pub;
std_msgs__msg__String sub_msg;
std_msgs__msg__Int32 pub_int_msg;
int pub_int_value;
std_msgs__msg__Int32 sub_int_msg;
int pub_string_value;
```

For the custom-defined trigger conditions, the type `pub_trigger_object_t` and the type `sub_trigger_object_t` are defined.

```C
typedef struct
{
  rcl_timer_t * timer1;
  rcl_timer_t * timer2;
} pub_trigger_object_t;

typedef struct
{
  rcl_subscription_t * sub1;
  rcl_subscription_t * sub2;
} sub_trigger_object_t;
```

The executor for the publishers shall publish when any of corresponding timers for the publishers is ready. That is the or-logic. You could also use the predefined  `rclc_executor_trigger_any` trigger condition, but this example shows how you can write your own trigger conditions.

In principle, the condition gets a list of handles, the length of this list, and the pre-defined condition type. In this case, we expect `pub_trigger_object_t`. First, the parameter `obj` is cased to this type (`comm_obj`). Then, each element of the handle list is checked for new data (or a timer is ready) by evaluating the field `handles[i].data_available` and its handle pointer is compared to the pointer of the communicatoin object. If at least one timer is ready, then the trigger condition returns true.

```C
bool pub_trigger(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  if (handles == NULL) {
    printf("Error in pub_trigger: 'handles' is a NULL pointer\n");
    return false;
  }
  if (obj == NULL) {
    printf("Error in pub_trigger: 'obj' is a NULL pointer\n");
    return false;
  }
  pub_trigger_object_t * comm_obj = (pub_trigger_object_t *) obj;
  bool timer1 = false;
  bool timer2 = false;
  //printf("pub_trigger ready set: ");
  for (unsigned int i = 0; i < size; i++) {
    if (handles[i].data_available) {
      void * handle_ptr = rclc_executor_handle_get_ptr(&handles[i]);
      if (handle_ptr == comm_obj->timer1) {
        timer1 = true;
      }
      if (handle_ptr == comm_obj->timer2) {
        timer2 = true;
      }
    }
  }
  return (timer1 || timer2);
}
```

The trigger condition for the subscription `sub_trigger`shall implement an AND-logic. That is, only if both subscriptions have received a new message, then the executor shall start processing the callbacks.

The implementation is analogous to `pub_trigger`. The only difference is, that this trigger returns true, if both subscriptions have been found in the handle list. This is implemented in the condition `sub1 && sub2` of the last if-statement.

```C
bool sub_trigger(rclc_executor_handle_t * handles, unsigned int size, void * obj)
{
  if (handles == NULL) {
    printf("Error in sub_trigger: 'handles' is a NULL pointer\n");
    return false;
  }
  if (obj == NULL) {
    printf("Error in sub_trigger: 'obj' is a NULL pointer\n");
    return false;
  }
  sub_trigger_object_t * comm_obj = (sub_trigger_object_t *) obj;
  bool sub1 = false;
  bool sub2 = false;
  //printf("sub_trigger ready set: ");
  for (unsigned int i = 0; i < size; i++) {
    if (handles[i].data_available == true) {
      void * handle_ptr = rclc_executor_handle_get_ptr(&handles[i]);

      if (handle_ptr == comm_obj->sub1) {
        sub1 = true;
      }
      if (handle_ptr == comm_obj->sub2) {
        sub2 = true;
      }
    }
  }
  return (sub1 && sub2);
}
```

Like in the Hello-World example, the subscription callbacks just prints out the received message.

The `my_string_subscriber` callback prints out the string of the message `msg->data.data`:

```C
void my_string_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("my_string_subscriber_callback: msgin is NULL\n");
  } else {
    printf("Callback 1: %s\n", msg->data.data);
  }
}
```

The integer callback prints out the received integer `msg->data`:

```C
void my_int_subscriber_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg == NULL) {
    printf("my_int_subscriber_callback: msgin is NULL\n");
  } else {
    printf("Callback 2: %d\n", msg->data);
  }
}
```

To publish messages with different frequencies, we setup two timers.
One timer to publish a string message, the `my_timer_string_callback` and one timer to publish the integer, the `my_timer_int_callback`.

In the `my_timer_string_callback`, the message `pub_msg` is created and filled with the string `Hello World` plus an integer, which is incremented by one, each time the timer callback is called. The the message is published with `rcl_publish()`

The macro `UNUSED` is a workaround for the linter warning, that the second parameter `last_call_time` is not used.

```C
#define UNUSED(x) (void)x;

void my_timer_string_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);

    std_msgs__msg__String pub_msg;
    std_msgs__msg__String__init(&pub_msg);
    const unsigned int PUB_MSG_CAPACITY = 20;
    pub_msg.data.data = malloc(PUB_MSG_CAPACITY);
    pub_msg.data.capacity = PUB_MSG_CAPACITY;
    snprintf(pub_msg.data.data, pub_msg.data.capacity, "Hello World!%d", pub_string_value++);
    pub_msg.data.size = strlen(pub_msg.data.data);

    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published: %s\n", pub_msg.data.data);
    } else {
      printf("Error in my_timer_string_callback: publishing message %s\n", pub_msg.data.data);
    }
    std_msgs__msg__String__fini(&pub_msg);
  } else {
    printf("Error in my_timer_string_callback: timer parameter is NULL\n");
  }
}
```

Likewise, the `my_timer_int_callback` increments the integer value `pub_int_value` in every call and assigns it to the message field `pub_int_msg.data`. Then the message is published with `rcl_publish()`

```C
void my_timer_int_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    pub_int_msg.data = pub_int_value++;
    rc = rcl_publish(&my_int_pub, &pub_int_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published: %d\n", pub_int_msg.data);
    } else {
      printf("Error in my_timer_int_callback: publishing message %d\n", pub_int_msg.data);
    }
  } else {
    printf("Error in my_timer_int_callback: timer parameter is NULL\n");
  }
}
```

Now were are all set for the `main()` function:

```C
int main(int argc, const char * argv[])
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }
```

First rcl is initialized with the `rclc_support_init` using the default `allocator`. The rclc-support objects are saved in `support`. Next, a node `my_node` with the name `node_0` and namespace `executor_examples` is created with:

```C
// create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&my_node, "node_0", "executor_examples", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }
```

A publisher `my_string_pub`, which publishes a string message and its corresponding timer `my_string_timer` with a 100ms period is created like this:

```C
// create a publisher 1
// - topic name: 'topic_0'
// - message type: std_msg::msg::String
const char * topic_name = "topic_0";
const rosidl_message_type_support_t * my_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

rc = rclc_publisher_init_default(&my_string_pub, &my_node, my_type_support, topic_name);
if (RCL_RET_OK != rc) {
  printf("Error in rclc_publisher_init_default %s.\n", topic_name);
  return -1;
}

// create timer 1
// - publishes 'my_string_pub' every 'timer_timeout' ms
rcl_timer_t my_string_timer = rcl_get_zero_initialized_timer();
const unsigned int timer_timeout = 100;
rc = rclc_timer_init_default(&my_string_timer, &support, RCL_MS_TO_NS(timer_timeout), my_timer_string_callback);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_timer_init_default.\n");
  return -1;
} else {
  printf("Created timer 'my_string_timer' with timeout %d ms.\n", timer_timeout);
}
```

Note that the previously defined `my_timer_string_callback` is connected to this timer.
Likewise, a second publisher `my_int_pub, which publishes an int message and its corresponding timer` my_int_timer` with 1000ms period, is created like this:

```C
// create publisher 2
  // - topic name: 'topic_1'
  // - message type: std_msg::msg::Int
  const char * topic_name_1 = "topic_1";
  const rosidl_message_type_support_t * my_int_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
  rc = rclc_publisher_init_default(&my_int_pub, &my_node, my_int_type_support, topic_name_1);
  if (RCL_RET_OK != rc) {
    printf("Error in rclc_publisher_init_default %s.\n", topic_name_1);
    return -1;
  }

  // create timer 2
  // - publishes 'my_int_pub' every 'timer_int_timeout' ms
  rcl_timer_t my_int_timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_int_timeout = 10 * timer_timeout;
  rc = rclc_timer_init_default(&my_int_timer, &support, RCL_MS_TO_NS(timer_int_timeout), my_timer_int_callback);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_timer_init_default.\n");
    return -1;
  } else {
    printf("Created timer with timeout %d ms.\n", timer_int_timeout);
  }
```

Note that the `my_timer_int_callback` is connected to the `my_int_timer`. The data variables used for the publisher messages in the timer callbacks need to be initialized first:

```C
std_msgs__msg__Int32__init(&int_pub_msg);
int_pub_value = 0;
string_pub_value = 0;
```

The first subscription `my_string_sub` is created with the function `rcl_subscription_init` because we change the quality-of-service parameter to 'last-is-best'. That is, a new message will overwrite the older message if it has not been processed by the subscription. Also the message `string_sub_msg` needs to be initialized.

```C
// create subscription 1
  rcl_subscription_t my_string_sub = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t my_subscription_options = rcl_subscription_get_default_options();
  my_subscription_options.qos.depth = 0; // qos: last is best = register semantics
  rc = rcl_subscription_init(&my_string_sub, &my_node, my_type_support, topic_name, &my_subscription_options);

  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }
  // initialize subscription message
  std_msgs__msg__String__init(&string_sub_msg);
```

The second subscription `my_int_sub` is created with the rclc convenience function `rclc_subscription_default` and the message `int_sub_msg` is properly initialized.

```C
// create subscription 2
  rcl_subscription_t my_int_sub = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(&my_int_sub, &my_node, my_int_type_support, topic_name_1);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name_1);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name_1);
  }
  // initialize subscription message
  std_msgs__msg__Int32__init(&int_sub_msg);
```

In this example, we are using two executors, one to schedule the publishers, and one to schedule the subscriptions:

```C
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
```

The executor `executor_pub` is first created with  `rclc_executor_get_zero_initialized_executor()` and has two handles (aka 2 timers).

```C
// Executor for publishing messages
  unsigned int num_handles_pub = 2;
  printf("Executor_pub: number of DDS handles: %u\n", num_handles_pub);
  executor_pub = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor_pub, &support.context, num_handles_pub, &allocator);

  rc = rclc_executor_add_timer(&executor_pub, &my_string_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer 'my_string_timer'.\n");
  }

  rc = rclc_executor_add_timer(&executor_pub, &my_int_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer 'my_int_timer'.\n");
  }
```

Both timers are added to the exececutor with the function `rclc_executor_add_timer`:

```C
rc = rclc_executor_add_timer(&executor_pub, &my_string_timer);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_timer 'my_string_timer'.\n");
}

rc = rclc_executor_add_timer(&executor_pub, &my_int_timer);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_timer 'my_int_timer'.\n");
}
```

Also the second publisher has two handles, the two subscriptions:

```C
unsigned int num_handles_sub = 2;
printf("Executor_sub: number of DDS handles: %u\n", num_handles_sub);
executor_sub = rclc_executor_get_zero_initialized_executor();
rclc_executor_init(&executor_sub, &support.context, num_handles_sub, &allocator);
```

Which are added with the function `rclc_executor_add_subscription`:

```C
// add subscription to executor
rc = rclc_executor_add_subscription(
  &executor_sub, &my_string_sub, &string_sub_msg,
  &my_string_subscriber_callback,
  ON_NEW_DATA);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_subscription 'my_string_sub'. \n");
}

// add int subscription to executor
rc = rclc_executor_add_subscription(
  &executor_sub, &my_int_sub, &int_sub_msg,
  &my_int_subscriber_callback,
  ON_NEW_DATA);
if (rc != RCL_RET_OK) {
  printf("Error in rclc_executor_add_subscription 'my_int_sub'. \n");
}
```

The trigger condition of the executor, which publishes messages, shall execute when any timer is ready. This can be configured with the function `rclc_executor_set_trigger` and the parameter `rclc_executor_trigger_any`.
While the executor for the subscriptions shall only execute if both messages have arrived. Therefore the trigger parameter `rclc_executor_trigger_any` can be used:

```C
rc = rclc_executor_set_trigger(&executor_pub, rclc_executor_trigger_any, NULL);
rc = rclc_executor_set_trigger(&executor_sub, rclc_executor_trigger_all, NULL);
```

Finally, the executors spin-some functions can be started. The sleep-time between the executors is intended for communication time for DDS.

```C
for (unsigned int i = 0; i < 100; i++) {
  // timeout specified in ns                 (here: 1s)
  rclc_executor_spin_some(&executor_pub, 1000 * (1000 * 1000));
  usleep(1000); // 1ms
  rclc_executor_spin_some(&executor_sub, 1000 * (1000 * 1000));
}
```

This example is concluded with the clean-up code:

```C
// clean up
rc = rclc_executor_fini(&executor_pub);
rc += rclc_executor_fini(&executor_sub);
rc += rcl_publisher_fini(&my_string_pub, &my_node);
rc += rcl_publisher_fini(&my_int_pub, &my_node);
rc += rcl_timer_fini(&my_string_timer);
rc += rcl_timer_fini(&my_int_timer);
rc += rcl_subscription_fini(&my_string_sub, &my_node);
rc += rcl_subscription_fini(&my_int_sub, &my_node);
rc += rcl_node_fini(&my_node);
rc += rclc_support_fini(&support);

std_msgs__msg__Int32__fini(&int_pub_msg);
std_msgs__msg__String__fini(&string_sub_msg);
std_msgs__msg__Int32__fini(&int_sub_msg);

if (rc != RCL_RET_OK) {
  printf("Error while cleaning up!\n");
  return -1;
}
return 0;
}
```

In case the default trigger conditions are not sufficient, then the user can define custom logic conditions.
The source code of the custom-programmed trigger condition has already been presented.
The following code will setup the executor accordingly:

```C
 pub_trigger_object_t comm_obj_pub;
 comm_obj_pub.timer1 = &my_string_timer;
 comm_obj_pub.timer2 = &my_int_timer;

 sub_trigger_object_t comm_obj_sub;
 comm_obj_sub.sub1 = &my_string_sub;
 comm_obj_sub.sub2 = &my_int_sub;

 rc = rclc_executor_set_trigger(&executor_pub, pub_trigger, &comm_obj_pub);
 rc = rclc_executor_set_trigger(&executor_sub, sub_trigger, &comm_obj_sub);
```

The custom structs `pub_trigger_object_t` are used to save the pointer of the handles. The timers `my_string_timer` and `my_int_timer` for the publishing executor; and, likewise, the subscriptions `my_string_sub` and `my_int_sub` for the subscribing executor. The configuration is done also with the `rclc_executor_set_trigger` by passing the trigger function and the trigger object, e.g. `pub_trigger` and `comm_obj_pub` for the `executor_pub`, respectivly.

The complete source code of this example can be found in the file [rclc-examples/example_executor_trigger.c](https://github.com/ros2/rclc/rclc_examples/example_executor_trigger.c).
