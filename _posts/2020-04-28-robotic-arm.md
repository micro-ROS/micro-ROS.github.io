---
title: Warehouses automation with micro-ROS
author: francesca-finocchiaro
---

Automation is the key to development within the supply chain and logistics sectors, as it has the potential to improve productivity while reducing costs and protecting workers’ health.

At its most basic, an automated warehouse attempts to cut down on manual tasks that slow down the movement of goods, and to minimize the exposure of human workers to potential hazards. As robotics evolves, ever more warehouses plan to increase their investment in technology, with a focus on automation and scheduling tools. 

Below, we present a demo that exemplifies the potential entailed by micro-ROS to act as a first-line player in the next generation of robotics applications in the logistics sector.

It showcases a ROS 2 enabled robotic arm (Robotis OpenMANIPULATOR-X) connected to a ST VL53l1X ToF sensor able to measure the distance between a target object and the base of the arm.
The sensor is operated by an Olimex STM32-E407 development board, which features a STM32F407 microcontroller running Zephyr, a Real-Time Operating System that is especially convenient thanks to the large collection of sensor drivers available. On top, runs a micro-ROS app that is in charge of passing the distance measurements to the software controlling the arm kinematics. This communication is mediated by a Raspberry Pi 4 bridge, through which the arm is instructed to grab the object with millimetric precision and relocate it in a different position.
  
Tasks of this kind can be integrated into a bigger and more complex operations chain, as building blocks of a fully automated protocol, relevant to sectors such as that of warehouses discussed above.

Find the documentation for reproducing this demo [here](https://github.com/micro-ROS/micro-ROS_openmanipulator_demo).

<iframe width="560" height="315" src="https://www.youtube.com/embed/10Gq-q8k2tw" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>