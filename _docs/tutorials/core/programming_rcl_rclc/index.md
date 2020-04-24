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



## <a name="services"/>Services

ROS 2 services is another communication mechanism between nodes. Services implement a client-server paradigm based on ROS 2 messages and types. Further information about ROS 2 services can be found [here](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)

Ready to use code related to this tutorial can be found in [`micro-ROS-demos/rcl/addtwoints_server`](https://github.com/micro-ROS/micro-ROS-demos/blob/dashing/rcl/addtwoints_server/main.c) and [`micro-ROS-demos/rcl/addtwoints_client`](https://github.com/micro-ROS/micro-ROS-demos/blob/dashing/rcl/addtwoints_client/main.c) folders.

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


## <a name="rclc_executor"/>Rclc Executor

