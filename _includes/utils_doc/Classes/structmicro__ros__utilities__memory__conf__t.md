# micro_ros_utilities_memory_conf_t

**Module:** **[micro-ROS Types Utilities](Modules/group__typeutils.md)**



Memory configuration string. 


`#include <type_utilities.h>`

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| size_t | **[max_string_capacity](Classes/structmicro__ros__utilities__memory__conf__t.md#variable-max_string_capacity)** <br>Maximum string capacity to use for msg fields in case they don't have a custom rule assigned to them.  |
| size_t | **[max_ros2_type_sequence_capacity](Classes/structmicro__ros__utilities__memory__conf__t.md#variable-max_ros2_type_sequence_capacity)** <br>Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain ROS 2 msg types, in case they don't have a custom rule assigned to them.  |
| size_t | **[max_basic_type_sequence_capacity](Classes/structmicro__ros__utilities__memory__conf__t.md#variable-max_basic_type_sequence_capacity)** <br>Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain basic types (ie: primitive field types), in case they don't have a custom rule assigned to them.  |
| const [micro_ros_utilities_memory_rule_t](Classes/structmicro__ros__utilities__memory__rule__t.md) * | **[rules](Classes/structmicro__ros__utilities__memory__conf__t.md#variable-rules)** <br>All rules defined in this configuration.  |
| size_t | **[n_rules](Classes/structmicro__ros__utilities__memory__conf__t.md#variable-n_rules)** <br>Total number of rules defined in this configuration.  |
| const rcutils_allocator_t * | **[allocator](Classes/structmicro__ros__utilities__memory__conf__t.md#variable-allocator)** <br>The allocator to use when applying this configuration.  |

## Public Attributes Documentation

### variable max_string_capacity

```cpp
size_t max_string_capacity;
```

Maximum string capacity to use for msg fields in case they don't have a custom rule assigned to them. 

### variable max_ros2_type_sequence_capacity

```cpp
size_t max_ros2_type_sequence_capacity;
```

Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain ROS 2 msg types, in case they don't have a custom rule assigned to them. 

### variable max_basic_type_sequence_capacity

```cpp
size_t max_basic_type_sequence_capacity;
```

Maximum capacity to use for sequence type msg fields (ie: unbounded arrays and lists) which contain basic types (ie: primitive field types), in case they don't have a custom rule assigned to them. 

### variable rules

```cpp
const micro_ros_utilities_memory_rule_t * rules;
```

All rules defined in this configuration. 

### variable n_rules

```cpp
size_t n_rules;
```

Total number of rules defined in this configuration. 

### variable allocator

```cpp
const rcutils_allocator_t * allocator;
```

The allocator to use when applying this configuration. 

-------------------------------