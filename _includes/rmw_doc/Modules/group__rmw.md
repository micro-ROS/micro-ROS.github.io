# micro-ROS RMW API

## Functions

|                | Name           |
| -------------- | -------------- |
| void | **[rmw_uros_set_continous_serialization_callbacks](#function-rmw_uros_set_continous_serialization_callbacks)**(rmw_publisher_t * publisher, rmw_uros_continous_serialization_size size_cb, rmw_uros_continous_serialization serialization_cb)<br>Sets the callback functions for continous serialization for a publisher.  |
| rmw_ret_t | **[rmw_uros_set_custom_transport](#function-rmw_uros_set_custom_transport)**(bool framing, void * args, open_custom_func open_cb, close_custom_func close_cb, write_custom_func write_cb, read_custom_func read_cb)<br>Sets micro-ROS default custom transport.  |
| rmw_ret_t | **[rmw_uros_options_set_custom_transport](#function-rmw_uros_options_set_custom_transport)**(bool framing, void * args, open_custom_func open_cb, close_custom_func close_cb, write_custom_func write_cb, read_custom_func read_cb, rmw_init_options_t * rmw_options)<br>Fills rmw implementation-specific options with the given custom transport.  |
| rmw_ret_t | **[rmw_uros_discover_agent](#function-rmw_uros_discover_agent)**(rmw_init_options_t * rmw_options)<br>Fills rmw implementation-specific options with the autodicovered address of an micro-ROS Agent.  |
| rmw_ret_t | **[rmw_uros_init_options](#function-rmw_uros_init_options)**(int argc, const char *const argv[], rmw_init_options_t * rmw_options)<br>Parses command line args and fills rmw implementation-specific options.  |
| rmw_ret_t | **[rmw_uros_options_set_serial_device](#function-rmw_uros_options_set_serial_device)**(const char * dev, rmw_init_options_t * rmw_options)<br>Fills rmw implementation-specific options with the given parameters.  |
| rmw_ret_t | **[rmw_uros_options_set_udp_address](#function-rmw_uros_options_set_udp_address)**(const char * ip, const char * port, rmw_init_options_t * rmw_options)<br>Fills rmw implementation-specific options with the given parameters.  |
| rmw_ret_t | **[rmw_uros_options_set_client_key](#function-rmw_uros_options_set_client_key)**(uint32_t client_key, rmw_init_options_t * rmw_options)<br>Fills rmw implementation-specific options with the given parameters.  |
| rmw_ret_t | **[rmw_uros_ping_agent](#function-rmw_uros_ping_agent)**(const int timeout_ms, const uint8_t attempts)<br>Check if micro-ROS Agent is up and running.  |
| rmw_ret_t | **[rmw_uros_ping_agent_options](#function-rmw_uros_ping_agent_options)**(const int timeout_ms, const uint8_t attempts, rmw_init_options_t * rmw_options)<br>Check if micro-ROS Agent is up and running using the transport set on the given rmw options.  |
| bool | **[rmw_uros_epoch_synchronized](#function-rmw_uros_epoch_synchronized)**()<br>Returns the time synchronization state of the epoch time.  |
| int64_t | **[rmw_uros_epoch_millis](#function-rmw_uros_epoch_millis)**()<br>Returns the epoch time in milliseconds taking into account the offset computed during the time synchronization.  |
| int64_t | **[rmw_uros_epoch_nanos](#function-rmw_uros_epoch_nanos)**()<br>Returns the epoch time in nanoseconds taking into account the offset computed during the time synchronization.  |
| rmw_ret_t | **[rmw_uros_sync_session](#function-rmw_uros_sync_session)**(const int timeout_ms)<br>Synchronizes the session time using the NTP protocol.  |
| rmw_ret_t | **[rmw_uros_set_publisher_session_timeout](#function-rmw_uros_set_publisher_session_timeout)**(rmw_publisher_t * publisher, int session_timeout)<br>Sets the DDS-XRCE session spin time in reliable publication.  |
| rmw_ret_t | **[rmw_uros_set_service_session_timeout](#function-rmw_uros_set_service_session_timeout)**(rmw_service_t * service, int session_timeout)<br>Sets the DDS-XRCE session spin time in reliable service server.  |
| rmw_ret_t | **[rmw_uros_set_client_session_timeout](#function-rmw_uros_set_client_session_timeout)**(rmw_client_t * client, int session_timeout)<br>Sets the DDS-XRCE session spin time in reliable service client.  |


## Functions Documentation

### function rmw_uros_set_continous_serialization_callbacks

```cpp
void rmw_uros_set_continous_serialization_callbacks(
    rmw_publisher_t * publisher,
    rmw_uros_continous_serialization_size size_cb,
    rmw_uros_continous_serialization serialization_cb
)
```

Sets the callback functions for continous serialization for a publisher. 

**Parameters**: 

  * **publisher** publisher where continous serialization is being configured 
  * **size_cb** callback that should modify the total serialization size 
  * **serialization_cb** callback that should serialize the user part of the message 


### function rmw_uros_set_custom_transport

```cpp
rmw_ret_t rmw_uros_set_custom_transport(
    bool framing,
    void * args,
    open_custom_func open_cb,
    close_custom_func close_cb,
    write_custom_func write_cb,
    read_custom_func read_cb
)
```

Sets micro-ROS default custom transport. 

**Parameters**: 

  * **framing** Enable XRCE framing. 
  * **args** Arguments for open function. 
  * **open_cb** Open transport callback. 
  * **close_cb** Close transport callback. 
  * **write_cb** Write transport callback. 
  * **read_cb** Read transport callback. 


**Return**: 

  * RMW_RET_OK If correct. 
  * RMW_RET_ERROR If invalid. 


### function rmw_uros_options_set_custom_transport

```cpp
rmw_ret_t rmw_uros_options_set_custom_transport(
    bool framing,
    void * args,
    open_custom_func open_cb,
    close_custom_func close_cb,
    write_custom_func write_cb,
    read_custom_func read_cb,
    rmw_init_options_t * rmw_options
)
```

Fills rmw implementation-specific options with the given custom transport. 

**Parameters**: 

  * **framing** Enable XRCE framing. 
  * **args** Arguments for open function. 
  * **open_cb** Open transport callback. 
  * **close_cb** Close transport callback. 
  * **write_cb** Write transport callback. 
  * **read_cb** Read transport callback. 
  * **rmw_options** Updated options with updated transport. 


**Return**: 

  * RMW_RET_OK If arguments were valid and set in rmw_init_options. 
  * RMW_RET_INVALID_ARGUMENT If rmw_init_options is not valid or unexpected arguments. 


### function rmw_uros_discover_agent

```cpp
rmw_ret_t rmw_uros_discover_agent(
    rmw_init_options_t * rmw_options
)
```

Fills rmw implementation-specific options with the autodicovered address of an micro-ROS Agent. 

**Parameters**: 

  * **rmw_options** Updated options with rmw specifics. 


**Return**: 

  * RMW_RET_OK If arguments were valid and set in rmw_init_options. 
  * RMW_RET_TIMEOUT If micro-ROS agent autodiscovery is timeout. 
  * RMW_RET_INVALID_ARGUMENT If rmw_init_options is not valid or unexpected arguments. 


### function rmw_uros_init_options

```cpp
rmw_ret_t rmw_uros_init_options(
    int argc,
    const char *const argv[],
    rmw_init_options_t * rmw_options
)
```

Parses command line args and fills rmw implementation-specific options. 

**Parameters**: 

  * **argc** Number of arguments. 
  * **argv** Arguments. 
  * **rmw_options** Updated options with rmw specifics. 


**Return**: 

  * RMW_RET_OK If arguments were valid and set in rmw_init_options. 
  * RMW_RET_INVALID_ARGUMENT If rmw_init_options is not valid or unexpected arguments. 


`rmw_init_options allocator` is used to allocate the specific rmw options.


### function rmw_uros_options_set_serial_device

```cpp
rmw_ret_t rmw_uros_options_set_serial_device(
    const char * dev,
    rmw_init_options_t * rmw_options
)
```

Fills rmw implementation-specific options with the given parameters. 

**Parameters**: 

  * **dev** Serial device. 
  * **rmw_options** Updated options with rmw specifics. 


**Return**: 

  * RMW_RET_OK If arguments were valid and set in rmw_init_options. 
  * RMW_RET_INVALID_ARGUMENT If rmw_init_options is not valid or unexpected arguments. 


### function rmw_uros_options_set_udp_address

```cpp
rmw_ret_t rmw_uros_options_set_udp_address(
    const char * ip,
    const char * port,
    rmw_init_options_t * rmw_options
)
```

Fills rmw implementation-specific options with the given parameters. 

**Parameters**: 

  * **ip** Agent IP address. 
  * **port** Agent UDP port. 
  * **rmw_options** Updated options with rmw specifics. 


**Return**: 

  * RMW_RET_OK If arguments were valid and set in rmw_init_options. 
  * RMW_RET_INVALID_ARGUMENT If rmw_init_options is not valid or unexpected arguments. 


### function rmw_uros_options_set_client_key

```cpp
rmw_ret_t rmw_uros_options_set_client_key(
    uint32_t client_key,
    rmw_init_options_t * rmw_options
)
```

Fills rmw implementation-specific options with the given parameters. 

**Parameters**: 

  * **client_key** MicroXRCE-DDS client key. 
  * **rmw_options** Updated options with rmw specifics. 


**Return**: 

  * RMW_RET_OK If arguments were valid and set in rmw_init_options. 
  * RMW_RET_INVALID_ARGUMENT If rmw_init_options is not valid or unexpected arguments. 


### function rmw_uros_ping_agent

```cpp
rmw_ret_t rmw_uros_ping_agent(
    const int timeout_ms,
    const uint8_t attempts
)
```

Check if micro-ROS Agent is up and running. 

**Parameters**: 

  * **timeout_ms** Timeout in ms (per attempt). 
  * **attempts** Number of tries before considering the ping as failed. 


**Return**: 

  * RMW_RET_OK If micro-ROS Agent is available. 
  * RMW_RET_ERROR If micro-ROS Agent is not available. 


This function can be called even when the micro-ROS context has not yet been initialized by the application logics. 


### function rmw_uros_ping_agent_options

```cpp
rmw_ret_t rmw_uros_ping_agent_options(
    const int timeout_ms,
    const uint8_t attempts,
    rmw_init_options_t * rmw_options
)
```

Check if micro-ROS Agent is up and running using the transport set on the given rmw options. 

**Parameters**: 

  * **timeout_ms** Timeout in ms (per attempt). 
  * **attempts** Number of tries before considering the ping as failed. 
  * **rmw_options** rmw options with populated transport parameters. 


**Return**: 

  * RMW_RET_OK If micro-ROS Agent is available. 
  * RMW_RET_ERROR If micro-ROS Agent is not available. 


This function can be called even when the micro-ROS context has not yet been initialized. The transport will be initialized and closed once during the ping process. 


### function rmw_uros_epoch_synchronized

```cpp
bool rmw_uros_epoch_synchronized()
```

Returns the time synchronization state of the epoch time. 

**Return**: true if last time synchronization succeded and false otherwise 

### function rmw_uros_epoch_millis

```cpp
int64_t rmw_uros_epoch_millis()
```

Returns the epoch time in milliseconds taking into account the offset computed during the time synchronization. 

**Return**: 

  * epoch time in milliseconds. 
  * 0 if session is not initialized. 


### function rmw_uros_epoch_nanos

```cpp
int64_t rmw_uros_epoch_nanos()
```

Returns the epoch time in nanoseconds taking into account the offset computed during the time synchronization. 

**Return**: 

  * epoch time in nanoseconds. 
  * 0 if session is not initialized. 


### function rmw_uros_sync_session

```cpp
rmw_ret_t rmw_uros_sync_session(
    const int timeout_ms
)
```

Synchronizes the session time using the NTP protocol. 

**Parameters**: 

  * **timeout_ms** The waiting time in milliseconds. 


**Return**: 

  * RMW_RET_OK when success. 
  * RMW_RET_ERROR If no session is running or the synchronization fails. 


### function rmw_uros_set_publisher_session_timeout

```cpp
rmw_ret_t rmw_uros_set_publisher_session_timeout(
    rmw_publisher_t * publisher,
    int session_timeout
)
```

Sets the DDS-XRCE session spin time in reliable publication. 

**Parameters**: 

  * **publisher** publisher where the spin time is configured 
  * **session_timeout** time in milliseconds 


**Return**: 

  * RMW_RET_OK when success. 
  * RMW_RET_INVALID_ARGUMENT If publisher is not valid or unexpected arguments. 


### function rmw_uros_set_service_session_timeout

```cpp
rmw_ret_t rmw_uros_set_service_session_timeout(
    rmw_service_t * service,
    int session_timeout
)
```

Sets the DDS-XRCE session spin time in reliable service server. 

**Parameters**: 

  * **service** service where the spin time is configured 
  * **session_timeout** time in milliseconds 


**Return**: 

  * RMW_RET_OK when success. 
  * RMW_RET_INVALID_ARGUMENT If service is not valid or unexpected arguments. 


### function rmw_uros_set_client_session_timeout

```cpp
rmw_ret_t rmw_uros_set_client_session_timeout(
    rmw_client_t * client,
    int session_timeout
)
```

Sets the DDS-XRCE session spin time in reliable service client. 

**Parameters**: 

  * **client** client where the spin time is configured 
  * **session_timeout** time in milliseconds 


**Return**: 

  * RMW_RET_OK when success. 
  * RMW_RET_INVALID_ARGUMENT If client is not valid or unexpected arguments. 






-------------------------------