---
title: Interoperability with FIWARE
permalink: /docs/concepts/fiware_interoperability/
---

## Motivation
Among the goals of the micro-ROS project, one of the key issues has been that of providing interoperability with third outstanding and broadly used platforms.
One of the selected components has been the FIWARE Context Broker, a standard for context data management adopted by several EU boosted initiatives for facilitating the development of smart solutions for different domains.

This section explains how to achieve interoperability between micro-ROS and this platform, passing through the integration of the latter with ROS 2.
Thanks to this interoperability, the FIWARE's Context Broker becomes micro-ROS' platform of choice for sharing context information with any other system integrated into the FIWARE ecosystem.

## Interoperability: pros and cons of the different possible solutions

This subsection will explain all the design alternatives for the interoperability between micro-ROS and the FIWARE Context Broker.
From now on, the developed solution for intercommunicating micro-ROS with FIWARE will be called **FIROS2 Integration Service**.

FIROS2 requires transformation libraries to convert ROS 2 messages into FIWARE NGSIv2 messages and the other way around.
For each message, one transformation library is required.

![image](http://www.plantuml.com/plantuml/svg/ZP712i8m38RlUOempuKvfrv49gYmap05BmCfhfs5hOMslhzjLuQYu1e8_E5Fyf4Mnb9jdtq77UCMhK8jseV5HcXsjq99uA9ZcA1xjQnEvmnxPWnjMIrzBK5giDpVvlXXF9RNNNNuRSqGf6f6guymr-sERHTDfU5AzzGJ39Rt2GkShJddQJeHBfyEj_o6YtQ75pRyWrkDS03XC8Hi1sW8ESeio1mtX0nT47AK3gDWil7_yW80)

In the implementation of these transformation libraries, it is required to be able to serialize/deserialize ROS 2 messages.
Also, an NGSIv2 serialisation/deserialisation mechanism will be used.

The FIROS2 package provides a standard NGSIv2 serialisation/deserialisation mechanisms, but ROS 2 serialisation/deserialisation is more complex, due to its dependencies with the message type.
Therefore, FIROS2 Integration Service needed to be implemented providing a simple user-wise solution to automatically generate the transformation libraries for ROS 2 types.

To accomplish this, two different approaches can be taken:
* Implementing an ad-hoc bridging communication tool for translating FIWARE's messages into micro-ROS (that is, ROS 2) messages types, and viceversa.
* Relying on an integration platform that uses a common types language representation, and defines a conversion library from/to the generic type to the specific type of each middleware.

While the first approach might result in a more lightweight tool, it has several flaws, for instance a more difficult maintenance and the incapability of communicating with any other middleware, rather than ROS2 or micro-ROS.
On the other hand, using an integration service platform, such as [SOSS](https://github.com/eProsima/Integration-Service), enables automatically the possibility of communicating with a wide (and growing) set of middlewares, if their System Handle implementation is available.

## SOSS: System-Of-Systems-Synthesizer

**SOSS** addresses the task of providing a common interface for communicating software platforms that speak different languages.
It is composed of a **core** library, which defines a set of abstract interfaces and provides some utility classes to form a plugin-based framework.

This pluggable interface allows the user to leverage any of the supported plugins or System Handles for a specific middleware, such as DDS, ROS2, FIWARE, or ROS, for the desired integration.

SOSS can act as an intermediate message-passing tool that, by speaking a common language, centralizes and mediates the integration of several applications running under different communication middlewares.
A SOSS instance is configured and launched by means of a **YAML** file, which allows the user to provide a mapping between the different topics and services that two or more applications can exchange information about.

Users can also develop their own System Handles for a new middleware, automatically granting communication capabilities with all the rest of supported middlewares.

Usually, types are defined using a common language representation, used by SOSS to create a shared representation of the exchanged information, so that it can be processed, converted and remapped to every middleware's types implementation, when required.
This common representation is provided, user-wise, using IDL definitions, which are parsed and converted into Dynamic Types representations at runtime, using [eProsima's XTypes-DDS](https://github.com/eProsima/xtypes) implementation.

<img width="600" src="imgs/soss.png">

## SOSS-FIWARE System Handle

The [FIWARE System Handle](https://github.com/eProsima/SOSS-FIWARE/tree/feature/xtypes-support) allows bringing information from and to FIWARE's Context Broker into the SOSS world.
This System Handle is configured and launched in the same way as any SOSS System Handle.

Besides the standard information included in any System Handle's configuration (such as system's name and type, which would be `fiware` for this specific System Handle),
in the case of the FIWARE System Handle users must specify two extra YAML key-value pairs, which are the host's IP and port in which this System Handle will try to connect to an instance of FIWARE's Orion Context Broker.

Regarding more specific details about the implementation, FIWARE does not allow certain characters in its entities names.
For this reason, if a type defined in the topics section of the configuration file has in its name a /, the FIWARE System Handle will map that character into two underscores.
This is something important to notice when connecting to ROS2, because in ROS2 most of the types have a / in their names.
To deal with this issue, using SOSS remapping capabilities come in handy.

Of course, given that micro-ROS applications act as a bridge between microcontrollers and the ROS 2 dataspace (using the micro-ROS Agent), FIROS2 should also take care of communicating FIWARE's Context Broker with ROS 2, leveraging the existing [ROS 2 System Handle](https://github.com/eProsima/Integration-Service), which comes natively included into the SOSS package.

This is exactly the situation reflected in the use case that is explained below.

## FIROS2 use case: connecting FIWARE with ROS 2

### Installation

* Create a *colcon* workspace.
  ```bash
    $ mkdir -p soss_ws/src
    $ cd soss_ws
  ```

* Clone the SOSS project into the source subfolder.
  ```bash
    $ git clone https://github.com/eProsima/Integration-Service.git src/soss --branch feature/xtypes-dds
  ```

* Clone the SOSS-FIWARE project into the source subfolder.
  ```bash
    $ git clone https://github.com/eProsima/SOSS-FIWARE.git src/soss-fiware --branch feature/xtypes-support
  ```

* The workspace layout should now look like this:
  ```bash
    soss_ws
      |
      |_ src
          |
          |_ soss
          |    |
          |    |_ (other soss project subfolders)
          |    |_ packages
          |          |
          |          |_ (other soss system handle subfolders)
          |          |_ soss-ros2 (ROS2 system handle)
          |
          |_ soss-fiware
                 |
                 |_ fiware (soss-fiware colcon package)
                 |_ fiware-test (soss-fiware-test colcon package)
  ```

* In the workspace folder, execute colcon.
  ```bash
    $ colcon build --packages-up-to soss-ros2 soss-fiware
  ```

* Source the resulting enviromnment:
  ```bash
    $ source install/local_setup.bash
  ```

### Configuration

SOSS must be configured with a YAML file, which tells the program everything it needs to know in order to establish the connection between two or more systems that the user wants.
For example, if the user wants to exchange a simple string message between FIWARE and ROS2, the configuration file for SOSS should look as follows:

  ```yaml
    systems:
        ros2: { type: ros2 }
        fiware: { type: fiware, host: CONTEXT_BROKER_IP, port: 1026}

    routes:
        fiware_to_ros2: { from: fiware, to: ros2 }
        ros2_to_fiware: { from: ros2, to: fiware }

    topics:
        hello_fiware: { type: "std_msgs/String", route: ros2_to_fiware }
        hello_ros2: { type: "std_msgs/String", route: fiware_to_ros2 }
  ```

In the project's *CMakeLists.txt* file, users must specify which ROS 2 packages are required for their SOSS instance,
by means of the provided `soss-rosidl-mix` macro in order to generate the proper IDL types definition and typesupport files resulting from ROS 2 *msg/srv* files:
  ```cmake
    soss_rosidl_mix(
      PACKAGES geometry_msgs nav_msgs test_msgs ...
      MIDDLEWARES ros2
    )
  ```

*Note:* if the package list is modified, it is recommended to re-build the whole colcon workspace. Otherwise, `.mix` files might not be generated for the new ROS 2 type package(s) included in the aforementioned macro.

Finally, after source the colcon workspace, you can launch FIROS2 with:
  ```bash
    cd <path_to_yaml_config_file>
    soss <yaml_config_file_name>.yaml
  ```
