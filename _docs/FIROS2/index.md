---
title: micro-ROS & FIWARE
permalink: /docs/FIROS2/
redirect_from: /FIROS2/
---


## Interoperability

This subsection will explain all the design alternatives for the interoperability of FIROS2 with micro-ROS.

### Mechanisms for the deserialisation of incoming data in the transformation library

FIROS2 requires transformation libraries to convert ROS2 messages into FIWARE NGSIv2 messages and the other way around.
For each message, one transformation library is required by the integration service (FIROS2).

![image](http://www.plantuml.com/plantuml/svg/ZP712i8m38RlUOempuKvfrv49gYmap05BmCfhfs5hOMslhzjLuQYu1e8_E5Fyf4Mnb9jdtq77UCMhK8jseV5HcXsjq99uA9ZcA1xjQnEvmnxPWnjMIrzBK5giDpVvlXXF9RNNNNuRSqGf6f6guymr-sERHTDfU5AzzGJ39Rt2GkShJddQJeHBfyEj_o6YtQ75pRyWrkDS03XC8Hi1sW8ESeio1mtX0nT47AK3gDWil7_yW80)

In the implementation of these transformation libraries, the user needs to be able to serialisation/deserialisation ROS2 messages.
Also, an NGSIv2 serialisation/deserialisation mechanism will be used.

The FIROS2 package provides a standard NGSIv2 serialisation/deserialisation mechanisms, but ROS2 serialisation/deserialisation is not offered due to its dependencies with the message type.

For solving this issue, various methods to get it are proposed:

#### Use serialisation/deserialisation method provided by the middleware layer

This is currently the method used in micro-ROS - FIROS 2 integration.

In this case, the transformation library will use user selected middleware interface to serialise/deserialise the bridged ROS2 messages.
This method requires to get the message typesupport for the bridged message type.
This method is straightforward to implement as it does not require additional source code development.
Also, the abstraction from the middleware implementation makes it more compatible with others ROS2 workspaces.

This is a portion of code used in the transformation library implementation.

```Cpp
    extern "C" void USER_LIB_EXPORT transform(SerializedPayload_t *serialized_input, SerializedPayload_t *serialized_output){

    // Get type support
    const rosidl_message_type_support_t * type_support = rosidl_typesupport_cpp::get_message_type_support_handle<MESAGE_TYPE>();

    // Convert to ROS2 serialized message
    rmw_serialized_message_t serialized_message;
    serialized_message.buffer = (char*)serialized_input->data;
    serialized_message.buffer_length = serialized_input->length;
    serialized_message.buffer_capacity = serialized_input->max_size;
    serialized_message.allocator = rcutils_get_default_allocator();

    // Deserialise
    MESAGE_TYPE data;
    if (rmw_deserialize(&serialized_message, type_support, (void*)&data) != RMW_RET_OK){
        return;
    }

    // Transformation and NGSIv2 serialisation code here

    }
```

Note the call to ROS 2 interface __rosidl_typesupport_cpp::get_message_type_support_handle__

#### Use serialisation/deserialisation method for an specific type support

In this case, the transformation library will use one specific type support to serialise/deserialise the bridged ROS2 messages.
In micro-ROS case, the implementation to be used will be rosidl_typesupport_microxrcedds.
This method is trivial to develop as it does not require additional source code on the micro-ROS side.

In the case of micro-ROS, the transformation library should use the serialisation/deserialisation API exposed by its typesupport, rosidl_typesupport_microxrcedds.
This mechanism requires the user to have access to the typesupport API, which sometimes is not always possible.

#### Used serialisation/deserialisation method generated from IDL file

In this case, transformation library will use generated code to serialise/deserialise the bridged ROS2 messages.
The generated code may be made using an IDL parser tool.
In the micro-ROS case, Micro XRCE-DDS provides with Micro XRCE-DDS code generator, which accepts an IDL file as input and generates type code.
This IDL files should correspond with those messages types the transformation is wanted.
This is the [integration service](https://github.com/eProsima/Integration-Service) native method.
Integration services uses this method, but it makes the development of the library slower as it needs to be generated per each message to be bridged.

![image](http://www.plantuml.com/plantuml/svg/bP8_2y8m4CNtV8f7tOIArd-R2DR1GGTT70eIqciRQ1D8qkzl4sr1iA0t11xlxjsF97lhk75jKxEQ2WUdOMHPEUJIa71IAwPqJeZGLQOoTPR2QDolXsESfZS8RvQao72dZM_mZH6unIbzB32XENN52baF8GrPoql20ZAluPtFgGIiGv855mvHfcwwDO9UcmfjC8ptsp2TYH1Z1rtrEbDzwX9V8P8HYDLl4Cb_4Ell49SHYCrl49V_8BPWZ8LxZkFTwvbOEDzo6UHgn5q7kHbnk-mzgTp_foS0)

In the case of ROS2/micro-ROS workspaces, there are tools which generate those IDL files.
The rosidl_gen package is the package micro-ROS/ROS2 could use to create IDL from ROS2 interfaces.

### Integration proposals

Aside from transformation library implementations possibilities, micro-ROS could be integrated into different levels with FIROS2.
This section presents all the integrations possibilities.

#### Direct integration

In this case, micro-ROS Agent will act as a bridge between DDS-XRCE and NGSIv2.

Selected bridged topics and their corresponding transformations must be configured on the micro-ROS Agent node.

This proposal requires micro-ROS Agent changes, but it is the direct native integration of micro-ROS and FIROS, without relying on DDS global data space.

**Architecture**

![image](http://www.plantuml.com/plantuml/png/TP1FImCn4CNFpgTux9vpSDlw1qHQM8KUL6WFNWJ99jCrT9j0TbOFudStKRkoXxwv98_tVYIpx4L76GuTTRmJI41qxPl0kiX6NF3KIuYwPHH8Ud0c1hLvseBzkul17zWBaWhe7klwzHoVT3PM_kC-M3vc5YYlBtT9z3Mbfs1r2boT0A_Q59pWBr0czfKr2M-wC5WKBpvFNM_noF8HulxNE3PcA7Lbx4AJrQ8RtNEkAALow7xzlDhSeObXpp4RsH-hSvJMv26Ydw_TA7Nxzugc6vZoSJHdcDxdj6Hlq_Q_0G00)

**Use case**

![image](http://www.plantuml.com/plantuml/png/jPNFZjCm48VFv2b6smiSigh_D4cbg5f45ua38EqUAK9kF6sZJMpaE1I4U7VieCgq3SAgkkd9NtvZVvrCcxlE2cFxjaaQt5Ym6aoztLcGjS7ArbebLQDx2JShjLBBvIDyGBlNvialRq1qy6xvXS1aCzsuAv72YhNeqCVJDFMXidphjjeBWx0s-WdDOk6nknise339cFyadTL6R5tzmyT72YlrSkTiqgzeDjgqGbK8gBxLHgiMbNrrg6VmCtbHA-jYedB5PJcKAorniJY4EFmxirBlwyher25ulKLb3qKvJ125d6BoAxY5h1Ey_suDjjZyW0ViT6ygX3TQTTQ8Mg64-n7T8aPt3l_Fa6bCY82Jlwon98jH9NcCHd59_obvZWT0wTdN2biUQrC6iKaELtMSnJjcqOxvTHsBUCVv-Cdnlt4UCufi1X6Xx99HPCLpZ2ARHxUGJw_wy3YBFtaOxMJugqy_xMefKNsMUgzIT_a07MvoA6zl5p34_7hszzg37Ceq3O6m3cy0Z-S7x0AJTTEZXsGwIiaP_UEPdxYGuaZ68qfET1mOzQ4iS1BEfdmSP-Cu7yVpS-mvEsgU1zbfDrbnuk_0g3yFhMD5E9hpSto7IlPjyni0)

#### Indirect integration with a single FIROS2 node

In this case, micro-ROS nodes will publish the configured topics on DDS, and a FIROS2 node will subscribe to those topics and convert them into NGSIv2 protocol.
Selected bridged topics must be configured on that single FIROS2 node.
Each ROS 2 topic type should have a corresponding transformation library configured on that FIROS2 node.

This approach is a specialisation of the following one, where all the configuration and transformation libraries are centralised in a single node.

This proposal requires transformation library development, but the integration will be the same as a regular ROS2 node, so no micro-ROS specific development should be expected.

**Architecture**

![image](http://www.plantuml.com/plantuml/png/TP71IyCm5CRFlh_YqPvpiDkiWiW6TT232jl1Yo1fybRBkWJILpt8_dVpfglTeUz1oFlo-pv2iknO1-uFBRIqOsIFeQa_66qJo73Z7NJiWwu94uprr9ZWrUPbY-G-c-3TWHpBGOAwmx9ulyPlk1ei_xZpbixC0jExV1SBZfVf4SocWhE9u5Kju3Z-1jEOVMlDY3ybhqjPnsW-e4SmhUyj9czEkYYs-4pyvSF-LpWxPfZgpDY51gjPLxeZiIYb15gNhwlD8rR1xoc88FfWdMDgZJG0d5xXNgc7lmjNRKyWsq6SeSpvv3o79JaRF-u7)

**Use Case**

![image](http://www.plantuml.com/plantuml/png/jLHTQzim57sUViMbUTaUN2mfJHmmeMDfeG_Te7rSnb2icyJKbeOiBnjZ_tsoMER4YPamgVsaetFknxaNtLPM65kN1IbmRS5gCFbcQq7c1ZERQqMoGjSIhfPggHQBP_Y8TgVDItEy0b71m-8hXT4wNhkFI675IbJOqACeQaXfUkz2xOH1M1dzWcO-Nof_smPWC9hmvYULrKPidFxfqpE3fNgxTTL4tz2ijIc5oX1GVS-DLYKg-swlv_2BlCcLTJIHEN6QUhdI4kVpFMaA_PobUimpeoC7mViDAhiN9J1253B6ZlRnI7p_q0XN9fSRt2jdC4eake_yRhjgixZMxdvOWoKMF-49ArqR5_c3LfKr8bSeuUvCepG-wRGDUTmkfU0p3_6JiX13AOS0qiqGs-can_V_sawdh-9x4kxx30APB8PBriXeS8sC1TV8XsyH6uTi4Hkq86msT45uVB0WbtEVJuFTvyb5vpuEd_kOGIZJpvtunptwlCsbHFL5wfsAtES7G0Zu-ocarzTpyCpcd40QHGVdMU-vVVNzS_KFJs2qAchqA3-Cxf6RJZuwwIIWqrwWa_AWj4cRayNdONOUvkVXep89yLZN2Xxt0ssbNikJzcRsDmtn4pt1FSnFuLjKiYwBFm00)

#### Indirect integration with multiple FIROS2 nodes

In this case, micro-ROS nodes will publish the configured topic on DDS and multiple FIROS2 nodes, one for each set topic, will subscribe to those topics and convert them into NGSIv2 protocol.

This approach would require more nodes on the network and individual configurations.

This approach is the one followed by micro-ROS, and it is limited due to current FIROS 2 implementation.

This proposal requires transformation library development, but the integration will be the same as a regular ROS2 node, so no micro-ROS specific development should be expected.

**Workflow**

![image](http://www.plantuml.com/plantuml/svg/ZPB1QiCm38RFqrE8vEn3qtPDO8mMia8Eww0zx38OhgrceQaDZhCTHjzziJTtQI1G2Oob_zCF5busbXlRdcgewM3HQZHL-M5HLeQ4hRI2nch3Iy88ktYkXD5i-x93Kf-LqUf4oZeXGjvWaRzFy1lkBYF_kDAI0ZF7E5iSke3pjNi79cF6oOZngdHWt_uUuyuxbQB7U-TruKw7uYJ0YnlW9C3f3V0cmDa5FeEeTIinUbCkyto76x9VsXn_6s5YYZ5FX9npaDpoFM_8ZJ367BGkNbVR9zmRVIJZ6huVHcSOI-4I0Fo67nXx_5l6lcu9_3KqfmZ-wMFrpDVfG4y7UZRGixw-92NTh_e1)

**Use Case**

![image](http://www.plantuml.com/plantuml/png/jPNFQzim5CVFor_ng2_RmIMIOyS6OpgM5dhO1krn6KEnNX9HHngoicoC_U-J38w3ApejfChfVVpz-lt8Gxvf3TDclsic3QuD60LQRBO6kD1O6w7af6xKdiFLKYxbOl48dz0Sb7vouHMm5kuNtmX4w-dQdWbdXgrOYquUnx4JbUMTq7XW6c6brHFgghBOFHrUG27A4lURj4Pfjh7-Xy-F59RoxTPM4tz7lLPwnsIDWFA7q4hkK9ftlJ-1tvCtGXehOxKrbsLdbufZLVMY6VnRaxwWMSgFejOgjWZURspwtfH1XCZu55_otjqxtqeBli7Uc4EKSqJyRsDnDFuP9ZPKPyTs-zDet67p2nwmLNjT8tnGiZMQ2OaSRZr39DDdKpo-SZGcGk2YcMbuvkIocMVXcLC8LMQsnZCtcSn3Lfda420gpbbcIi_TPfgynbRIy7-8fX2gp8ALSxXeDtbuDuNBR4ztmSPVje9pb5-vEiZOdaxIFPn1UNrTGITpSJgACoZZk8yTo4_149_UmmuN8rdXbn5ov1b4gsQF7KsyDFNIp4lpKH-aE0LT9vIEQgI91_Ygfkd0wP2KPduyz-FYENRd1YMtNzI_)

## Demonstration

This section explains how to demonstrate the interoperability of FIROS2 with microROS.
The purpose is to demonstrate the interoperability, although the final design is not closed.

To run the demonstration a step by step guide is presented in this document.

> **Note:** The only requirement to run the demonstration is to have [docker CE](https://docs.docker.com/install/) and [docker compose](https://docs.docker.com/compose/install/) installed.

### Linux demonstration

1. **Run Micro-ROS Agent node**

Download the pre-compiled agent and run it

```shell
docker pull microros/agent_linux
docker run  -it --rm  --privileged --net=host microros/agent_linux
```

Once inside the docker, raise the agent.

```shell
uros_agent udp 8888
```

> **Note:** After this step a micro-ROS Agent will be running at the localhost address and port 8888.

1. **Run a FIWARE Orion broker**

To test the communication it is necessary to have a FIWARE Orion server listening. The server will be run locally using a docker compose.
The steps have been extracted from the docker hub [official FIWARE repository](https://hub.docker.com/r/fiware/orion).

For this, execute the following commands in a terminal and leave it open.

```shell
(
mkdir orion
cd orion
echo "mongo:
    image: mongo:3.4
    command: --nojournal
orion:
    image: fiware/orion
    links:
        - mongo
    ports:
        - \"1026:1026\"
    command: -dbhost mongo" > docker-compose.yml
sudo docker-compose up
)
```

> **Note:** After this execution a FIWARE Orion server will be running at the localhost address and port 1026.

1. **Build FIROS2 in a ROS2 workspace and run it**

To compile FIROS2, micro-ROS Agent side set of packages will be used.

For this, execute the following instructions.

```shell
(
sudo docker pull microros/linux
sudo docker run  -it --rm  --privileged --net=host microros/linux
)
```

Once in the Docker, all the necessary repositories should be downloaded and a FIROS2 node built and configured as a gateway of an int32.

```shell
(
mkdir -p ws/src
cd ws
wget https://raw.githubusercontent.com/microROS/micro-ROS-doc/feature/RepoListUpdate/Installation/repos/agent_minimum.repos
vcs import src < agent_minimum.repos
git clone -b feature/FIROS2 https://github.com/microROS/micro-ROS-demos.git src/uros/Demos
git clone --recursive -b feature/TCP_DynTypes https://github.com/eProsima/FIROS2.git src/uros/FIROS2
colcon build
. ./install/local_setup.bash
install/firos2/bin/firos2 install/int32_firos2/lib/int32_firos2/config.xml
)
```

1. **Run Micro-ROS client publisher**

Download a pre-compiled client and execute it.

```shell
docker pull microros/client_linux
docker run  -it --rm  --privileged --net=host microros/client_linux
```

Once in the docker run the microROS Client.

```shell
int32_publisher_c
```

1. **Check that data has been uploaded into FIWARE Orion server**

In a Linux terminal execute the below sub-shell script

```shell
(
    UPDATE_TIME="0.5"

    curl  -v \
        --include \
        --header 'Content-Type: application/json' \
        --request POST \
        --data-binary '{  "id": "Helloworld",
                            "type": "Helloworld",
                            "$ATTRIBUTE": {
                                "value": "0",
                                "type": "Number"
                            }
                        }' \
        'localhost:1026/v2/entities'

    (
        while (( 1 ))
        do
        curl -v "localhost:1026/v2/entities/Helloworld/attrs/count/value?type=Helloworld"
        echo ""
        sleep $UPDATE_TIME
        done
    )
)
```

For further information please refer to the official FIROS2 documentation: [FIROS2 documentation](https://github.com/eProsima/FIROS2)
