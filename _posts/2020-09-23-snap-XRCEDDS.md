---
title: Micro XRCE-DDS Agent now on snap!
author: francesca-finocchiaro
---

In collaboration with [Canonical](https://canonical.com/), we took the liberty to wrap the [Micro XRCE-DDS Agent](https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html) as a [snap](https://snapcraft.io/) package, which sets an important milestone for easing the use of Micro XRCE-DDS.

<img alt="Canonical" src="/img/posts/canonical.png" width="35%"/>

Snap is a package manager designed to bundle and handle applications and their dependencies on several Linux distros, among which Ubuntu.

<img alt="Snap" src="/img/posts/snap2.png" width="45%"/>

The snap packaging of the Agent comes with two ways of running it: as a simple executable or by means of a Linux service.

The first implies the usage of the Agent’s built-in CLI, where you can specify the standard configuration parameters (such as transport, port…) directly, as follows:
```
micro-xrce-dds-agent <transport> <args>
```

The second launches the Agent as a Linux service running in the background; to do so, simply execute:
```
snap set micro-xrce-dds-agent daemon=true
```
In this case, instead of specifying the Agent’s launch parameters via the CLI, users can configure them thanks to the `snap services` interface, using the `snap set micro-xrce-dds-agent <param>=<value>` command. These parameters’ values are only valid when the `daemon` option is set to `true`, while they are ignored in `snap run` mode.

To have a look at the full list of configurable parameters, click [here](https://snapcraft.io/micro-xrce-dds-agent).

Given the closeness of the two Agents libraries, a snap release of the micro-ROS Agent is just one step away!

{% include logos_disclaimer.md %}
