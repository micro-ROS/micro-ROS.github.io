# micro-ROS String Utilities

## Functions

|                | Name           |
| -------------- | -------------- |
| MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String | **[micro_ros_string_utilities_init](#function-micro_ros_string_utilities_init)**(const char * data)<br>Create a rosidl_runtime_c__String from a char pointer.  |
| MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String | **[micro_ros_string_utilities_init_with_size](#function-micro_ros_string_utilities_init_with_size)**(const size_t size)<br>Create a rosidl_runtime_c__String from a size.  |
| MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String | **[micro_ros_string_utilities_set](#function-micro_ros_string_utilities_set)**(const rosidl_runtime_c__String str, const char * data)<br>Create a rosidl_runtime_c__String from a char pointer.  |
| const MICRO_ROS_UTILITIES_PUBLIC char * | **[micro_ros_string_utilities_get_c_str](#function-micro_ros_string_utilities_get_c_str)**(const rosidl_runtime_c__String str)<br>Returns the char pointer to the rosidl_runtime_c__String data.  |
| rosidl_runtime_c__String | **[micro_ros_string_utilities_append](#function-micro_ros_string_utilities_append)**(const rosidl_runtime_c__String str, const char * data)<br>Appends a char pointer to the end of a rosidl_runtime_c__String.  |
| MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String | **[micro_ros_string_utilities_remove_tail_chars](#function-micro_ros_string_utilities_remove_tail_chars)**(const rosidl_runtime_c__String str, const size_t n)<br>Removes characters from the end of a string.  |
| MICRO_ROS_UTILITIES_PUBLIC void | **[micro_ros_string_utilities_destroy](#function-micro_ros_string_utilities_destroy)**(rosidl_runtime_c__String *const str)<br>Destroys a rosidl_runtime_c__String.  |


## Functions Documentation

### function micro_ros_string_utilities_init

```cpp
MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String micro_ros_string_utilities_init(
    const char * data
)
```

Create a rosidl_runtime_c__String from a char pointer. 

**Parameters**: 

  * **data** char pointer 


**Return**: `rosidl_runtime_c__String` string containing data 



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_string_utilities_init_with_size

```cpp
MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String micro_ros_string_utilities_init_with_size(
    const size_t size
)
```

Create a rosidl_runtime_c__String from a size. 

**Parameters**: 

  * **size** size of the required string 


**Return**: `rosidl_runtime_c__String` string of size size 



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_string_utilities_set

```cpp
MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String micro_ros_string_utilities_set(
    const rosidl_runtime_c__String str,
    const char * data
)
```

Create a rosidl_runtime_c__String from a char pointer. 

**Parameters**: 

  * **str** rosidl_runtime_c__String to set 
  * **data** char pointer 


**Return**: `rosidl_runtime_c__String` string containing data 



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_string_utilities_get_c_str

```cpp
const MICRO_ROS_UTILITIES_PUBLIC char * micro_ros_string_utilities_get_c_str(
    const rosidl_runtime_c__String str
)
```

Returns the char pointer to the rosidl_runtime_c__String data. 

**Parameters**: 

  * **str** a rosidl_runtime_c__String 


**Return**: `const char` char pointer 



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_string_utilities_append

```cpp
rosidl_runtime_c__String micro_ros_string_utilities_append(
    const rosidl_runtime_c__String str,
    const char * data
)
```

Appends a char pointer to the end of a rosidl_runtime_c__String. 

**Parameters**: 

  * **str** a rosidl_runtime_c__String 
  * **data** characters to append 


**Return**: `rosidl_runtime_c__String` new string 



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_string_utilities_remove_tail_chars

```cpp
MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String micro_ros_string_utilities_remove_tail_chars(
    const rosidl_runtime_c__String str,
    const size_t n
)
```

Removes characters from the end of a string. 

**Parameters**: 

  * **str** a rosidl_runtime_c__String 
  * **n** number of characters to remove 


**Return**: `rosidl_runtime_c__String` new string 



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_string_utilities_destroy

```cpp
MICRO_ROS_UTILITIES_PUBLIC void micro_ros_string_utilities_destroy(
    rosidl_runtime_c__String *const str
)
```

Destroys a rosidl_runtime_c__String. 

**Parameters**: 

  * **str** a rosidl_runtime_c__String 




------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |






-------------------------------