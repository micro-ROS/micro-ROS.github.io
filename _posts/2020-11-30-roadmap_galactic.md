---
title: Roadmap for ROS 2 Galactic
author: francesca-finocchiaro
---

After reviewing our most recent accomplishments and dusting off our To-Do list, we drafted the roadmap for the next months of development and designed an exciting plan for the ROS 2 Galactic release.

Our priorities will be:

- Migrating all functionalities to the `rclc`, so as to convert it into the default micro-ROS user API and an independent abstraction layer on top of the  `rcl`. Namely, the missing features to achieve this goal are:
  - Implement parameters in `rclc`, together with a dedicated parameters manager in the Agent.
  - Migrate actions to the `rclc`, as we recently did with services.
  - Migrate graphs to the `rclc`, based on their recent implementation both in the `rmw` and in the Agent.
- Adapt micro-ROS packages and CI to the rolling release before Galactic is out.
- Implement and polish a mature peer-to-peer functionality to achieve brokerless communication among Clients.

Additional improvements: continue to improve the stability and extend the capability of the middleware layer, evaluating which efforts are worthy to be addressed (e.g. implement multi-threading).
