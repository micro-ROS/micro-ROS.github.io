---
title: micro-ROS utilities
permalink: /docs/tutorials/programming_rcl_rclc/micro-ROS/
---

<!-- TODO: Change section name -->

<img src="https://img.shields.io/badge/Written_for-Humble-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Rolling-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Iron-green" style="display:inline"/>

- [Allocators](#allocators)
  - [Custom allocator](#custom-allocator)
- [Time sync](#time-sync)
- [Ping agent](#ping-agent)
- [Continous serialization](#continous-serialization)

## Allocators

  The allocator object wraps the dynamic memory allocation and deallocating methods used in micro-ROS

  ```c
  // Get micro-ROS default allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();
  ```

  The default allocator wraps the following methods:

  ```c
  - allocate = wraps malloc()
  - deallocate = wraps free()
  - reallocate = wraps realloc()
  - zero_allocate = wraps calloc()
  - state = `NULL`
  ```

### Custom allocator

Working in embedded systems, the user might need to modify this default functions with its own memory allocation methods.
To archieve this, the user can modify the default allocator with its own methods:

```c
// Get empty allocator
rcl_allocator_t custom_allocator = rcutils_get_zero_initialized_allocator();

// Set custom allocation methods
custom_allocator.allocate = microros_allocate;
custom_allocator.deallocate = microros_deallocate;
custom_allocator.reallocate = microros_reallocate;
custom_allocator.zero_allocate =  microros_zero_allocate;

// Set custom allocator as default
if (!rcutils_set_default_allocator(&custom_allocator)) {
    ... // Handle error
    return -1;
}
```

Custom methods prototypes and examples:

- allocate:

  Allocates memory given a size, an error should be indicated by returning `NULL`:

  ```c
  // Function prototype:
  void * (*allocate)(size_t size, void * state);

  // Implementation example:
  void * microros_allocate(size_t size, void * state){
    (void) state;
    void * ptr = malloc(size);
    return ptr;
  }
  ```

- deallocate

  Deallocate previously allocated memory, mimicking free():

  ```c
  // Function prototype:
  void (* deallocate)(void * pointer, void * state);

  // Implementation example:
  void microros_deallocate(void * pointer, void * state){
    (void) state;
    free(pointer);
  }
  ```

- reallocate:

  Reallocate memory if possible, otherwise it deallocates and allocates:

  ```c
  // Function prototype:
  void * (*reallocate)(void * pointer, size_t size, void * state);

  // Implementation example:
  void * microros_reallocate(void * pointer, size_t size, void * state){
    (void) state;
    void * ptr = realloc(pointer, size);
    return ptr;
  }
  ```

- zero_allocate:

  Allocate memory with all elements set to zero, given a number of elements and their size. An error should be indicated by returning `NULL`:

  ```c
  // Function prototype:
  void * (*zero_allocate)(size_t number_of_elements, size_t size_of_element, void * state);

  // Implementation example:
  void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state){
    (void) state;
    void * ptr = malloc(number_of_elements * size_of_element);
    memset(ptr, 0, number_of_elements * size_of_element);
    return ptr;
  }
  ```

  *Note: the `state` input argument is espected to be unused*

## Time sync
micro-ROS Clients can synchronize their epoch time with the connected Agent, this can be very useful when working in embedded environments that do not provide any time synchronization mechanism.
This utility is based on the NTP protocol, taking into account delays caused by the transport layer. An usage example can be found on [`micro-ROS-demos/rclc/epoch_synchronization`](https://github.com/micro-ROS/micro-ROS-demos/blob/iron/rclc/epoch_synchronization/main.c).

```c
// Sync timeout
const int timeout_ms = 1000;

// Synchronize time with the agent
rmw_uros_sync_session(timeout_ms);

if (rmw_uros_epoch_synchronized())
{
    // Get time in milliseconds or nanoseconds
    int64_t time_ms = rmw_uros_epoch_millis();
    int64_t time_ns = rmw_uros_epoch_nanos();
}
```

## Ping agent
The Client can test the connection with the Agent with the ping utility. This functionality can be used even when the micro-ROS context has not yet been initialized, which is useful to test the connection before trying to connect to the Agent. An example can be found on [`micro-ROS-demos/rclc/ping_uros_agent`](https://github.com/micro-ROS/micro-ROS-demos/blob/iron/rclc/ping_uros_agent/main.c).

```c
// Timeout for each attempt
const int timeout_ms = 1000;

// Number of attemps
const uint8_t attemps = 5;

// Ping the agent
rmw_ret_t ping_result = rmw_uros_ping_agent(timeout_ms, attempts);

if (RMW_RET_OK == ping_result)
{
    // micro-ROS Agent is reachable
    ...
}
else
{
    // micro-ROS Agent is not available
    ...
}
```

*Note: `rmw_uros_ping_agent` is thread safe.*

## Continous serialization

This utility allows the client to serialize and send data up to a customized size. The user can set the topic lenght and then serialize the data within the publish process. An example can be found on [`micro-ROS-demos/rclc/ping_uros_agent`](https://github.com/micro-ROS/micro-ROS-demos/blob/iron/rclc/ping_uros_agent/main.c), where fragments from an image are requested and serialized on the spot.

The user needs to define two callbacks, then set them on the `rmw`. It is recommended to clean the callbacks after the publication, to avoid interferences with other topics on the same process:

```c
// Set serialization callbacks
rmw_uros_set_continous_serialization_callbacks(size_cb, serialization_cb);

// Publish message
rcl_publish(...);

// Clean callbacks
rmw_uros_set_continous_serialization_callbacks(NULL, NULL);
```

- Size callback:

This callback will pass a pointer with the calculated message size. The user is responsible of increase this size to the expected value:

```c
// Function prototype:
void (* rmw_uros_continous_serialization_size)(uint32_t * topic_length);

// Implementation example:
void size_cb(uint32_t * topic_length){
    // Increase message size
    *topic_length += ucdr_alignment(*topic_length, sizeof(uint32_t)) + sizeof(uint32_t);
    *topic_length += IMAGE_BYTES;
}
```

- Serialize callback:

This callback gives the user the message buffer to be completed. The user is responsible of serialize the data up to the lenght established on the size callback:

```c
// Function prototype:
void (* rmw_uros_continous_serialization)(ucdrBuffer * ucdr);

// Implementation example:
void serialization_cb(ucdrBuffer * ucdr){
    size_t len = 0;
    micro_ros_fragment_t fragment;

    // Serialize array size
    ucdr_serialize_uint32_t(ucdr, IMAGE_BYTES);

    while(len < IMAGE_BYTES){
      // Wait for new image "fragment"
      ...

      // Serialize data fragment
      ucdr_serialize_array_uint8_t(ucdr, fragment.data, fragment.len);
      len += fragment.len;
    }
}
```

*Note: When the callback ends, the message will be published.*
