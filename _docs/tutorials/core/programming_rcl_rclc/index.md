---
title: Programming with rcl and rclc
permalink: /docs/tutorials/core/programming_rcl_rclc/
---

In this tutorial, you'll learn the basics of the micro-ROS C API. The major concepts (publishers, subscriptions, services,timers, ...) are identical with ROS 2. They even rely on the *same* implementation, as the micro-ROS C API is based on the ROS 2 client support library (rcl), enriched with a set of convenience functions by the package [rclc](https://github.com/micro-ROS/rclc/). That is, rclc does not add a new layer of types on top of rcl (like rclcpp and rclpy do) but only provides functions that ease the programming with the rcl types. New types are introduced only for concepts that are missing in rcl, such as the concept of an executor.

* [Creating a node](#node)
* [Publishers and subscriptions](#pub_sub)
* [Services](#services)
* [Timers](#timers)
* [Rclc Executor](#rclc_executor)


## <a name="node"/>Creating a Node

To simplify the creation of a node with rcl, rclc provides two functions `rclc_support_init(..)` and `rclc_node_init_default(..)` in [rclc/init.h](https://github.com/micro-ROS/rclc/blob/master/rclc/include/rclc/init.h) and [rclc/node.h](https://github.com/micro-ROS/rclc/blob/master/rclc/include/rclc/node.h), respectively. The first lines of the main function of a micro-ROS programm are:

```c
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

Creating a publisher by `rclc_publisher_init_default(..)` from [rclc/publisher.h](https://github.com/micro-ROS/rclc/blob/master/rclc/include/rclc/publisher.h):

```c
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

Publishing a message:

```c
std_msgs__msg__String__init(&pub_msg);
const unsigned int PUB_MSG_SIZE = 20;
char pub_string[PUB_MSG_SIZE];
snprintf(pub_string, 13, "%s", "Hello World!");
rosidl_generator_c__String__assignn(&pub_msg, pub_string, PUB_MSG_SIZE);
```

Creating a subscription by `rclc_subscription_init_default(..)` from [rclc/subscription.h](https://github.com/micro-ROS/rclc/blob/master/rclc/include/rclc/subscription.h), respectively:

```c
rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
rc = rclc_subscription_init_default(&my_sub, &my_node, &my_type_support, &my_topic_name);
if (rc != RCL_RET_OK) {
  printf("Failed to create subscriber.\n");
  return -1;
}
```

## <a name="services"/>Services

ROS 2 services is another communication mechanism between nodes. Services implement a client-server paradigm based on ROS 2 messages and types. Further information about ROS 2 services can be found [here](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)

Ready to use code related to this tutorial can be found in [`micro-ROS-demos/rcl/addtwoints_server`](https://github.com/micro-ROS/micro-ROS-demos/blob/dashing/rcl/addtwoints_server/main.c) and [`micro-ROS-demos/rcl/addtwoints_client`](https://github.com/micro-ROS/micro-ROS-demos/blob/dashing/rcl/addtwoints_client/main.c) folders.

Note: Services are not supported in rclc package yet. Therefore the configuration is described using RCL layer.

Starting from a code where RCL is initialized and a micro-ROS node is created, these steps are required in order to generate a service server:

```c
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

```c
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

```
int64 a
int64 b
---
int64 sum
```

Once service client and server are configured, service client can perform a request and wait for reply:

```c
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

```c
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
`rclc_timer_init_default(..)` in [rclc/timer.h](https://github.com/micro-ROS/rclc/blob/master/rclc/include/rclc/timer.h):
```c
// create a timer, which will call the publisher with period=`timer_timeout` ms in the 'my_timer_callback'
rcl_timer_t my_timer = rcl_get_zero_initialized_timer();
const unsigned int timer_timeout = 1000;
rc = rclc_timer_init_default(
  &my_timer,
  &support,
  RCL_MS_TO_NS(timer_timeout),
  my_timer_callback);
if (rc != RCL_RET_OK) {
  printf("Error in rcl_timer_init_default.\n");
  return -1;
} else {
  printf("Created timer with timeout %d ms.\n", timer_timeout);
}
```

## <a name="rclc_executor"/>Rclc Executor
The Executor is configured and setup with the functions
`rclc_executor_get_zero_initialized_executor(..)`, `rclc_executor_set_timeout(..)`, `rclc_executor_add_subscription(...)`, `rclc_executor_add_timer(...)`, `rclc_executor_spin_some(...)` and `rclc_executor_fini(...)` in [rclc/executor.h](https://github.com/micro-ROS/rclc/blob/master/rclc/include/rclc/executor.h).

Assuming that you have created a subscription and a timer, as described above, the following code snippet shows the configuration of the RCLC-executor.

The executor is configured with 2 handles, aka one subscription and one timer. The `support.context` is the ROS 2 context, as shown above with `rclc_support_init(...)`.

The timeout is the waiting time for new handles to get ready, for example new messages arrive or timers are ready. The unit of the timeout is milliseconds.

A subscription `my_sub`, a place-holder for it's message `sub_msg` and it's corresponding callback `my_subscriber_callback` is added to the executor with the function `rclc_executor_add_subscription(...)`

A timers is added to the executor with the function `rclc_executor_add_timer(...)`

A key feature of the rclc-Executor is, that the order of these functions to add handles matters. It defines the processing order when multiple messages have arrived or timers are ready. This provides full control over the execution order to the user.

There are are several functions, how to start the executor, one is `rclc_executor_spin_some(...)`, which allows the user to specify a timeout in nanoseconds.

Finally the code snippet shows how to destroy the objects for the node, publisher, subscriber and the executor with the respective fini-methods.

```c
rclc_executor_t executor;

  // compute total number of subsribers and timers
  unsigned int num_handles = 1 + 1;
  printf("Debug: number of DDS handles: %u\n", num_handles);
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

  // set timeout for rcl_wait()
  unsigned int rcl_wait_timeout = 1000;   // in ms
  rc = rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_set_timeout.");
  }

  // add subscription to executor
  rc = rclc_executor_add_subscription(&executor, &my_sub, &sub_msg, &my_subscriber_callback,
      ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_subscription. \n");
  }

  rclc_executor_add_timer(&executor, &my_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer.\n");
  }

  for (;;) {
    // timeout specified in ns (here 1s)
    rclc_executor_spin_some(&executor, 1000 * (1000 * 1000));
  }

  // clean up
  rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_timer_fini(&my_timer);
  rc += rcl_subscription_fini(&my_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  ```
