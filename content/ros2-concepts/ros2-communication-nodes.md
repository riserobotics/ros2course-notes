+++
title = "Communication between nodes"
weight = 4
+++

To make sure that data that is gathered, for example, on the camera is being transmitted to the actuator so that the robot would physically react to an obstacle that is in front of him, ROS2 provides a **communication system** so that different nodes can communicate with another. This communication system, also somethimes referred to as a data-distribution layer, will send information between different processes at runtime.