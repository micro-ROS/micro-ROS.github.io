---
title: Creating custom static micro-ROS library 
permalink: /docs/tutorials/advanced/create_custom_static_library/
---

<img src="https://img.shields.io/badge/Applies_to-all_current_distros-green" style="display:inline"/>

This tutorial aims at providing step-by-step guidance for those users interested in compiling micro-ROS as a standalone library in order to integrate it in custom development tools.

This tutorial starts in a previously created micro-ROS environment. Check the first steps of [**First micro-ROS application on an RTOS**](../../core/first_application_rtos/) for instructions on how to create a micro-ROS environment for embedded platforms.

Once your micro-ROS workspace is created and the `micro_ros_setup` tool is installed, we are going to prepare the micro-ROS environment:

```bash
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
```

Once all the packages are downloaded, we need to create a couple of files in order to crosscompile a custom static library and a set of header files:

```bash
touch my_custom_toolchain.cmake
touch my_custom_colcon.meta
```

## Example of a CMake toolchain

For example for a Cortex M3 a sample toolchain could be:

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# SET HERE THE PATH TO YOUR C99 AND C++ COMPILERS
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

# SET HERE YOUR BUILDING FLAGS
set(FLAGS "-O2 -ffunction-sections -fdata-sections -fno-exceptions -mcpu=cortex-m3 -nostdlib -mthumb --param max-inline-insns-single=500 -DF_CPU=84000000L -D'RCUTILS_LOG_MIN_SEVERITY=RCUTILS_LOG_MIN_SEVERITY_NONE'" CACHE STRING "" FORCE)

set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++11 ${FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

set(__BIG_ENDIAN__ 0)
```

## Example of a colcon meta file

A sample colcon.meta file with micro-ROS external transports could be:

```json
{
    "names": {
        "tracetools": {
            "cmake-args": [
                "-DTRACETOOLS_DISABLED=ON",
                "-DTRACETOOLS_STATUS_CHECKING_TOOL=OFF"
            ]
        },
        "rosidl_typesupport": {
            "cmake-args": [
                "-DROSIDL_TYPESUPPORT_SINGLE_TYPESUPPORT=ON"
            ]
        },
        "rcl": {
            "cmake-args": [
                "-DBUILD_TESTING=OFF",
                "-DRCL_COMMAND_LINE_ENABLED=OFF",
                "-DRCL_LOGGING_ENABLED=OFF"
            ]
        }, 
        "rcutils": {
            "cmake-args": [
                "-DENABLE_TESTING=OFF",
                "-DRCUTILS_NO_FILESYSTEM=ON",
                "-DRCUTILS_NO_THREAD_SUPPORT=ON",
                "-DRCUTILS_NO_64_ATOMIC=ON",
                "-DRCUTILS_AVOID_DYNAMIC_ALLOCATION=ON"
            ]
        },
        "microxrcedds_client": {
            "cmake-args": [
                "-DUCLIENT_PIC=OFF",
                "-DUCLIENT_PROFILE_UDP=OFF",
                "-DUCLIENT_PROFILE_TCP=OFF",
                "-DUCLIENT_PROFILE_DISCOVERY=OFF",
                "-DUCLIENT_PROFILE_SERIAL=OFF",
                "-UCLIENT_PROFILE_STREAM_FRAMING=ON",
                "-DUCLIENT_PROFILE_CUSTOM_TRANSPORT=ON"
            ]
        },
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=5",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
                "-DRMW_UXRCE_MAX_SERVICES=1",
                "-DRMW_UXRCE_MAX_CLIENTS=1",
                "-DRMW_UXRCE_MAX_HISTORY=4",
                "-DRMW_UXRCE_TRANSPORT=custom"
            ]
        }
    }
}
```

## Building the custom library

Once you have both files ready, just run the build step in the micro-ROS build system:

```bash
ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_custom_toolchain.cmake $(pwd)/my_custom_colcon.meta
```

Once the build finishes you will have a precompiled static library with all the micro-ROS functionality in `firmware/build/libmicroros.a` and you will have all the required headers for your application in `firmware/build/include`. 

Just use them to link against in your development tools, and remember **if you are using a commercially available board we are accepting micro-ROS ports from the community**.
