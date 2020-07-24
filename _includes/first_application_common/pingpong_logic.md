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

![pingpong](http://www.plantuml.com/plantuml/png/ZOwnIWGn48RxFCNFzSkoUG2vqce5jHEHi1dtWZkPa6GByNntavZY10yknMJu-ORlFwPiOjvvK-d3-M2YOR1uMKvHc93ZJafvoMML07d7h1NAE-DPWblg_na8vnwEx9OeZmzFOt1-BK7AzetJciPxCfRYVw1S0SbRLBEg1IpXPIvpUWLCmZpXIm6BS3addt7uQpu0ZQlxT1MK2r0g-7sfqbsbRrVfMrMwgbev3CDTlsqJGtJhATUmSMrMg5TKwaZUxfcttuMt7m00)