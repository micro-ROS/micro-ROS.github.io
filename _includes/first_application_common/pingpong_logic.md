This example showcases a micro-ROS node with two publisher-subscriber pairs associated with a `ping` and a `pong`
topics, respectively.
The node sends a `ping` package with a unique identifier, using a `ping` publisher.
If the `ping` subscriber receives a `ping` from an external node, the `pong` publisher responds to the incoming `ping`
with a `pong`. To test that this logic is correctly functioning, we implement communication with a ROS 2 node that:

* Listens to the topics published by the `ping` subscriber.
* Publishes a `fake_ping` package, that is received by the micro-ROS `ping` subscriber.
  As a consequence, the `pong` publisher on the micro-ROS application will publish a `pong`, to signal that it received
  the `fake_ping` correctly.

The diagram below clarifies the communication flow between these entities:

![pingpong](https://www.plantuml.com/plantuml/png/ZOv1IyGm48Nl-HMFlPVI_W3PewSAwar4GZjsWvrCIAO7SVtlTc8FAxYmbydZyONl7Olwh2ilhdo4c7ps39OeuoaB4pIlv5oKYV0oRBTx1Np1qE7B0QDmaaXHSMWvZ5aU7vxQ5E9y02fdkRiAoWKe1dvVglfTrT-kwczLzQQguz0qT_lVUj7aC9_KoihLMw4wqGOgGIL1tZ6O43ZZML8OxVrCXFDU_jsv5KMdDovpQU_9JvJ_0-KAI762cPqxRd5LNdu3Bpy0)