---
title: Services
permalink: /docs/tutorials/programming_rcl_rclc/service/
---

ROS 2 services are another communication mechanism between nodes. Services implement a client-server paradigm based on ROS 2 messages and types. Further information about ROS 2 services can be found [here](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)

Ready to use code related to this tutorial can be found in [`micro-ROS-demos/rclc/addtwoints_server`](https://github.com/micro-ROS/micro-ROS-demos/blob/foxy/rclc/addtwoints_server/main.c) and [`micro-ROS-demos/rclc/addtwoints_client`](https://github.com/micro-ROS/micro-ROS-demos/blob/foxy/rclc/addtwoints_client/main.c) folders. Fragments of code from this examples are used on this tutorial.

## <a name="server"/>Service server

### <a name="server_init"/>Initialization
Starting from a code where RCL is initialized and a micro-ROS node is created, there are tree ways to initialize a service server:
  
// TODO: explain and link diferences between each approach on QoS section
  
- Reliable (default):  

  ```C
  // Service server object
  rcl_service_t service;
  const char * service_name = "/addtwoints";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Initialize server with default configuration
  rcl_ret_t rc = rclc_service_init_default(&service, &node, type_support, service_name);

  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```
  
  A reliable service server will wait for confirmation for each response sended, which can increase the blocking time of the executor spins, `rmw-microxrcedds` offers an API to configure the acknowledgement timeout for each service:

  ```C
  // Set confirmation timeout in milliseconds
  int ack_timeout = 1000; 
  rc = rmw_uros_set_service_session_timeout(&service, ack_timeout);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```

  The default value for all clients is configured at compilation time by the cmake variable `RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT`.
  
- Best effort:  

  ```C
  // Service server object
  rcl_service_t service;
  const char * service_name = "/addtwoints";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Initialize server with default configuration
  rcl_ret_t rc = rclc_service_init_best_effort(&service, &node, type_support, service_name);

  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```
  
- Custom QoS:  

  ```C
  // Service server object
  rcl_service_t service;
  const char * service_name = "/addtwoints";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Set service QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_services_default;

  // Initialize server with customized quality-of-service options
  rcl_ret_t rc = rclc_service_init(&service, &node, type_support, service_name, qos_profile);

  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```
  
### <a name="server_callback"/>Callback

// TODO: add and explain function prototype?
Once a request arrives, the executor will call the configured callback with the request and response messages as arguments.
The request message contains the values sended by the client, the response_msg should be modified here as it will be delivered after the callback returns.

Using `AddTwoInts.srv` type definition as an example:

```C
int64 a
int64 b
---
int64 sum
```

The client request message will contain two integers `a` and `b`, and expectes the `sum` of them as a response:

```C
void service_callback(const void * request_msg, void * response_msg){
  // Cast messages to expected types
  example_interfaces__srv__AddTwoInts_Request * req_in = (example_interfaces__srv__AddTwoInts_Request *) request_msg;
  example_interfaces__srv__AddTwoInts_Response * res_in = (example_interfaces__srv__AddTwoInts_Response *) response_msg;

  // Handle request message and set the response message values
  printf("Client requested sum of %d and %d.\n", (int) req_in->a, (int) req_in->b);
  res_in->sum = req_in->a + req_in->b;
}
```
  
Note that it is neccesary to cast each message to the expected type

Once the service and the executor are initialized, the service callback must be added to the executor in order to process incoming requests once the executor is spinning:

```C
// Service message objects
example_interfaces__srv__AddTwoInts_Response response_msg;
example_interfaces__srv__AddTwoInts_Request request_msg;

// Add server callback to the executor
rc = rclc_executor_add_service(&executor, &service, &request_msg, &response_msg, service_callback);

// Spin executor to receive requests
rclc_executor_spin(&executor);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}

// TODO ??
// rclc_executor_add_service_with_context
// rclc_executor_add_service_with_request_id
```
  
## <a name="client"/>Service Client

### <a name="client_init"/>Initialization
The service client initialization is almost identical to the server one:

- Reliable (default):

  ```C
  // Service client object
  rcl_client_t client;
  const char * service_name = "/addtwoints";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Initialize client with default configuration
  rcl_ret_t rc = rclc_client_init_default(&client, &node, type_support, service_name);

  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```
  
  A reliable service client will wait for confirmation for each request sended, which can increase the blocking time of the executor spins, `rmw-microxrcedds` offers an API to configure the acknowledgement timeout for each client:

  ```C
  // Set confirmation timeout in milliseconds
  int ack_timeout = 1000; 
  rc = rmw_uros_set_client_session_timeout(&client, ack_timeout);

  if (RCL_RET_OK != rc) {
    ...  // Handle error
    return -1;
  }
  ```
  
  The default value for all clients is configured at compilation time by the cmake variable `RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT`.

- Best effort:  

  ```C
  // Service client object
  rcl_client_t client;
  const char * service_name = "/addtwoints";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Initialize client with default configuration
  rcl_ret_t rc = rclc_client_init_best_effort(&client, &node, type_support, service_name);

  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```
  
- Custom QoS:  
  
  ```C
  // Service client object
  rcl_client_t client;
  const char * service_name = "/addtwoints";

  // Get message type support
  const rosidl_message_type_support_t * type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  // Set client QoS
  const rmw_qos_profile_t * qos_profile = &rmw_qos_profile_services_default;

  // Initialize server with customized quality-of-service options
  rcl_ret_t rc = rclc_client_init(&client, &node, type_support, service_name, qos_profile);

  if (rc != RCL_RET_OK) {
    ...  // Handle error
    return -1;
  }
  ```
  
### <a name="client_callback"/>Callback
The executor is responsible to call the configured callback when the service response arrives. 
The function will have the response message as its only argument, containing the values sended by the server.

It is neccesary to cast the response message to the expected type. Example:
```C
void client_callback(const void * response_msg){
  // Cast response message to expected type
  example_interfaces__srv__AddTwoInts_Response * msgin = (example_interfaces__srv__AddTwoInts_Response * ) response_msg;

  // Handle response message
  printf("Received service response %ld + %ld = %ld\n", req.a, req.b, msgin->sum);
}
```

Once the client and the executor are initialized, the client callback must be added to the executor in order to receive the service response once the executor is spinning:

```C
// Response message object
example_interfaces__srv__AddTwoInts_Response res;

// Add client callback to the executor
rcl_ret_t rc = rclc_executor_add_client(&executor, &client, &res, client_callback);

if (rc != RCL_RET_OK) {
  ...  // Handle error
  return -1;
}

// Spin executor to receive requests
rclc_executor_spin(&executor);
```

### <a name="client_send"/>Send a request
Once the service client and server are configured, the service client can perform a request and spin the executor to get the reply.
Following the example on `AddTwoInts.srv`:

```C
// Request message object (Must match initialized client type support)
example_interfaces__srv__AddTwoInts_Request request_msg;

// Initialize request message memory and set its values
example_interfaces__srv__AddTwoInts_Request__init(&request_msg);
request_msg.a = 24;
request_msg.b = 42;

// Sequence number of the request (Populated in rcl_send_request)
int64_t sequence_number; 

// Send request
rcl_send_request(&client, &request_msg, &sequence_number);

// Spin the executor to get the response
rclc_executor_spin(&executor);
```

### <a name="services_end"/>Cleaning Up

To destroy an initialized service or client:

```C
// Destroy service server and client
rcl_service_fini(&service, &node);
rcl_client_fini(&client, &node);
```

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.
