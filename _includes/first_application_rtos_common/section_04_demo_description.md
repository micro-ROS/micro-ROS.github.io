These reference examples consist in the ping-pong app, from the [First micro-ROS application on Linux](../../first_application_linux/) tutorial. In this app, a micro-ROS node sends a ping message with a unique identifier using a publisher. The message is received by pong subscribers (in another ROS 2 or micro-ROS node). The ping-pong node will also answer to pings received from other nodes with a pong message, as elucidated by the diagram below:

![pingpong](http://www.plantuml.com/plantuml/png/ZOwnIWGn48RxFCNFzSkoUG2vqce5jHEHi1dtWZkPa6GByNntavZY10yknMJu-ORlFwPiOjvvK-d3-M2YOR1uMKvHc93ZJafvoMML07d7h1NAE-DPWblg_na8vnwEx9OeZmzFOt1-BK7AzetJciPxCfRYVw1S0SbRLBEg1IpXPIvpUWLCmZpXIm6BS3addt7uQpu0ZQlxT1MK2r0g-7sfqbsbRrVfMrMwgbev3CDTlsqJGtJhATUmSMrMg5TKwaZUxfcttuMt7m00)

The files that a micro-ROS app needs are RTOS-dependent, but they differ only in the includes and signature of the main function. The remaining lines are RTOS-independent micro-ROS code. The table below clarifies which files are required for creating a new app in the three RTOSes supported:

<table >
    <thead>
        <tr>
            <th></th>
            <th>File</th>
            <th>Description</th>
            <th></th>
        </tr>
    </thead>
   <tr>
    <td rowspan="4">Nuttx</td>
    <td >app.c</td>
    <td >micro-ROS app code.</td>
    <td rowspan="4"><a href="https://github.com/micro-ROS/apps/tree/dashing/examples/uros_pingpong">Sample app</a></td>
  </tr>
  <tr>
    <td >Kconfig</td>
    <td >Nuttx Kconfig configuration</td>
  </tr>
  <tr>
    <td>Make.defs</td>
    <td>Nuttx build system definitions</td>
  </tr>
  <tr>
    <td >Makefile</td>
    <td >Nuttx specific app build script</td>
  </tr>
  <tr>
    <td rowspan="2">FreeRTOS</td>
    <td >app.c</td>
    <td >micro-ROS app code.</td>
    <td rowspan="2"><a href="https://github.com/micro-ROS/freertos_apps/tree/dashing/apps/ping_pong">Sample app</a></td>

  </tr>
  <tr>
    <td >app-colcon.meta</td>
    <td >micro-ROS app specific colcon configuration. Detailed info <a href="https://micro-ros.github.io/docs/tutorials/core/microxrcedds_rmw_configuration/">here</a>.</td>
  </tr>
  <tr>
    <td rowspan="4">Zephyr</td>
    <td >src/app.c</td>
    <td >micro-ROS app code.</td>
    <td rowspan="4"><a href="https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong">Sample app</a></td>
  </tr>
  <tr>
    <td >app-colcon.meta</td>
    <td >micro-ROS app specific colcon configuration. Detailed info <a href="https://micro-ros.github.io/docs/tutorials/core/microxrcedds_rmw_configuration/">here</a>.</td>
  </tr>
  <tr>
    <td>CMakeLists.txt</td>
    <td>CMake file for Zephyr app building</td>
  </tr>
  <tr>
    <td >prj.conf</td>
    <td >Zephyr specific app configuration</td>
  </tr>
</table>

The following steps are RTOS-specific commands for creating a new app once the firmware folder is created inside `microros_ws`. Please refer to the table above in order to get an idea of the content that the RTOS-specific files described below needed to create your own app should contain.
