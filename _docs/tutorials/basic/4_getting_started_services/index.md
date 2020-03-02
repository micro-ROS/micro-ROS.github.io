ROS 2 services is another communication mechanism between nodes. Services implement a client-server paradigms based on ROS 2 messages and types. Further information about ROS 2 services can be found [here](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)

Ready to use code related to this tutorial can be found in `micro-ROS-demos/rcl/addtwoints_server` and `micro-ROS-demos/rcl/addtwoints_client` folders.

Starting from a code where RCL is init and a micro-ROS node is created, these steps are required in order to generate a service server:

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

First of all, seeing `AddTwoInts.srv` type definition it is possible to determine request and reply elements of the service. Service client will make a request with two integers and service server should send its sum as a response.

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
