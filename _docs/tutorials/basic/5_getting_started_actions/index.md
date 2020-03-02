Actions are one of the three communication mechanism between nodes available on ROS 2. Detailed information about ROS2 actions can be found [here](http://design.ros2.org/articles/actions.html).

Ready to use code related to this tutorial can be found in `micro-ROS-demos/rcl/fibonacci_action_server` and `micro-ROS-demos/rcl/fibonacci_action_client` folders.

Along this example, a simple action based on the well-known Fibonacci sequence is going to be implemented. First of all, let's suppose that there is a configured ROS 2 node.

## Action server

From this point an action server is going to be instantiated and the wait set is going to be created with the help of the RCL Actions API:

```c
// Creating action server, options and clock
rcl_action_server_t action_server = rcl_action_get_zero_initialized_server();
rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

rcl_clock_t clock;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_ros_clock_init(&clock, &allocator)

// Initializing action server
rcl_action_server_init(&action_server, &node, &clock, ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci), "fibonacci", &action_server_ops);

// Retrieving the number of ROS 2 entities required by the action server
size_t num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services;
rcl_action_server_wait_set_get_num_entities(&action_server, &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services);

// Creating the wait set
rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
rcl_wait_set_init(&wait_set, num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services, 0, &context, rcl_get_default_allocator());
```

Once the action server is created, let's create a function in charge of performing the proposed action: a delayed version of a Fibonacci sequence calculator:

```c
typedef struct fibonacci_args {
  uint32_t order;
  bool * goal_done;
  int32_t * feedback;
  int32_t * feedback_lenght;
} fibonacci_args;

void * fibonacci(void *args){
  fibonacci_args * fib_args = (fibonacci_args *)args;

  *fib_args->goal_done = false;

  fib_args->feedback[0] = 0;
  fib_args->feedback[1] = 1;
  *fib_args->feedback_lenght = 2;

  for (uint32_t i = 2; i <= fib_args->order; ++i) {

    *fib_args->feedback_lenght = i;
    fib_args->feedback[i] = fib_args->feedback[i-1] + fib_args->feedback[i-2];
    
    usleep(500000);
  }

  *fib_args->goal_done = true;
}
```

Along the ROS 2 node main loop, it will check the action server status using the RCL wait function and the RCL Action API utilities:

```c
rcl_wait_set_clear(&wait_set);

size_t index;
rcl_action_wait_set_add_action_server(&wait_set, &action_server, &index);

rcl_wait(&wait_set, RCL_MS_TO_NS(200));

// Requesting actions events from the wait set
bool is_goal_request_ready = false;
bool is_cancel_request_ready = false;
bool is_result_request_ready = false;
bool is_goal_expired = false;

rcl_action_server_wait_set_get_entities_ready(&wait_set, &action_server, &is_goal_request_ready, &is_cancel_request_ready, &is_result_request_ready, &is_goal_expired);
```

Although ROS 2 actions allows a complete state machine related to the lifecycle of actions goals, this example uses the simplest approach: the action server will process only one goal at once, rejecting goal requests received in between. So, let's see how to accept or reject a goal request:

```c
if (is_goal_request_ready) { 
  // Taking the goal request
  rmw_request_id_t request_header;
  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;
  rcl_action_take_goal_request(&action_server, &request_header, &ros_goal_request);

  // Sending goal request response
  example_interfaces__action__Fibonacci_SendGoal_Response ros_goal_response;
  ros_goal_response.accepted = !processing_goal; // Reject goal if there is another goal running
  rcl_action_send_goal_response(&action_server, &request_header, &ros_goal_response);

  if (ros_goal_response.accepted) {
    // Updating and publishing goal status of the action server

    goal_info = rcl_action_get_zero_initialized_goal_info();
    goal_info.goal_id = ros_goal_request.goal_id;
    goal_handle = rcl_action_accept_new_goal(&action_server, &goal_info);
    
    rcl_action_update_goal_state(goal_handle, GOAL_EVENT_EXECUTE);

    rcl_action_goal_status_array_t c_status_array = rcl_action_get_zero_initialized_goal_status_array();
    rcl_action_get_goal_status_array(&action_server, &c_status_array);
    rcl_action_publish_status(&action_server, &c_status_array.msg);

    // Launching the goal processing thread
    processing_goal = true;
    feedback = (int32_t*) malloc(ros_goal_request.goal.order * sizeof(int32_t));
    goal_order = ros_goal_request.goal.order;

    fibonacci_args args = {
      .order = ros_goal_request.goal.order,
      .goal_done = &goal_done,
      .feedback = feedback,
      .feedback_lenght = &feedback_lenght,
    };

    pthread_create(&goal_thread, NULL, fibonacci, &args);
  }
}
```

Once the goal is accepted and the goal processing thread is running, action server is supposed to send feedback about the goal processing state:

```c
if (!goal_done && processing_goal) {
  // Publish feedback

  example_interfaces__action__Fibonacci_FeedbackMessage ros_goal_feedback; 

  ros_goal_feedback.goal_id = goal_info.goal_id;
  ros_goal_feedback.feedback.sequence.data = feedback;
  ros_goal_feedback.feedback.sequence.size = feedback_lenght;
  ros_goal_feedback.feedback.sequence.capacity = goal_order;

  rcl_action_publish_feedback(&action_server, &ros_goal_feedback);
}
```

Once goal process is done, action server must notify that the goal processing has been successful:

```c
if (goal_done && processing_goal) {
  // Notifying goal success
  processing_goal = false;
  rcl_action_update_goal_state(goal_handle, GOAL_EVENT_SUCCEED);
  rcl_action_notify_goal_done(&action_server);
}
```

Finally, the action client request the goal result:

```c
if (is_result_request_ready && goal_done && !processing_goal) {
  // Sending goal result
  goal_done = false;

  example_interfaces__action__Fibonacci_GetResult_Request ros_result_request;
  rmw_request_id_t request_header;
  rcl_action_take_result_request(&action_server, &request_header, &ros_result_request);

  example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;

  ros_result_response.result.sequence.capacity = goal_order;
  ros_result_response.result.sequence.size = feedback_lenght;
  ros_result_response.result.sequence.data = feedback;
  ros_result_response.status = action_msgs__msg__GoalStatus__STATUS_SUCCEEDED;

  rcl_action_send_result_response(&action_server, &request_header, &ros_result_response);

  free(feedback);
}
```


## Action client

As before, let's start creating a ROS 2 action client supposing that there is already a ROS 2 node created:

```c
// Creating action client and options
rcl_action_client_t action_client = rcl_action_get_zero_initialized_client();
rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();

// Initializing action client
rcl_action_client_init(&action_client, &node, ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci), "fibonacci", &action_client_ops);

// Retrieving the number of ROS 2 entities required by the action client
size_t num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services;
rcl_action_client_wait_set_get_num_entities(&action_client, &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services);
  
// Creating the wait set
rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
rv = rcl_wait_set_init(&wait_set, num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services, 0, &context, rcl_get_default_allocator());
```

Before starting the ROS 2 node main loop, let's send a goal request:

```c
int64_t goal_sequence_number;
int64_t result_sequence_number;

int order = 10;

example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;
ros_goal_request.goal.order = order;
rv = rcl_action_send_goal_request(&action_client, &ros_goal_request, &goal_sequence_number);
```

In the same way that was done with the server the RCL Actions API is used to check actions events in the node main loop:

```c
rcl_wait_set_clear(&wait_set);
    
size_t client_index, subscription_index;
rcl_action_wait_set_add_action_client(&wait_set, &action_client, &client_index, &subscription_index);

rcl_wait(&wait_set, RCL_MS_TO_NS(50));

// Requesting actions events from the wait set
bool is_feedback_ready = false;
bool is_status_ready = false;
bool is_goal_response_ready = false;
bool is_cancel_response_ready = false;
bool is_result_response_ready = false;

rcl_action_client_wait_set_get_entities_ready(&wait_set, &action_client, &is_feedback_ready, &is_status_ready, &is_goal_response_ready, &is_cancel_response_ready, &is_result_response_ready);
```

If goal request has been accepted, let's request the goal result:

```c
if (is_goal_response_ready){
  example_interfaces__action__Fibonacci_SendGoal_Response ros_goal_response;
  rmw_request_id_t response_header;

  rcl_action_take_goal_response(&action_client, &response_header, &ros_goal_response);

  if (ros_goal_response.accepted)
  {
    goal_accepted = true;

    example_interfaces__action__Fibonacci_GetResult_Request ros_result_request;
    rcl_action_send_result_request(&action_client, &ros_result_request, &result_sequence_number);
  }
}
```

During the execution of the goal, action client can check its subscription to feedback:

```c
if(is_feedback_ready){
  example_interfaces__action__Fibonacci_FeedbackMessage ros_feedback;

  ros_feedback.feedback.sequence.data = (int32_t*) malloc(order * sizeof(int32_t));
  ros_feedback.feedback.sequence.capacity = order;

  rcl_action_take_feedback(&action_client, &ros_feedback);
  
  printSequence(ros_feedback.feedback.sequence);
 
  free(ros_feedback.feedback.sequence.data);
} 
```

Once the goal result is ready, action client can retrieve it:

```c
if(is_result_response_ready){
  example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;
  rmw_request_id_t response_header;

  ros_result_response.result.sequence.data = (int32_t*) malloc(order * sizeof(int32_t));
  ros_result_response.result.sequence.capacity = order;

  rcl_action_take_result_response(&action_client, &response_header, &ros_result_response);
  
  printSequence(ros_result_response.result.sequence);

  free(ros_result_response.result.sequence.data);
}
```