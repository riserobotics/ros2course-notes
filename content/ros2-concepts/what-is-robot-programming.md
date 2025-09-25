+++
title = "What is robot programming?"
weight = 2
+++

#### A little bit of History!
ROS stands for Robot Operating System, yet it is not an operating system. Most accurately, we may describe ROS2 as a collection of tools, libraries, middleware and a community of packages that can be used to control some robots. The federated structure, and the modular nature of the system allows users to interchange software packages inbetween projects, and make components reusable.

Traditionally this was, and still is not always the most straight forward approach. Robots are by nature quite heterogenous machines, in that two robots must not share any two components or even remotely look alike. This makes robot programming hard and equally heterogenous, people and companies usually started by developing robot software specifically tailored to their own needs, which is in many cases the right approach for companies with large R&D budgets and many people involved in development.

For academia and startup environments however, this would mean a massive investment of time and resources into the development of boilerplate code to enable robots to do basic tasks, reinventing the wheel many times over with no real gain of performance. For this audience [Willow Garage](https://en.wikipedia.org/wiki/Willow_Garage), a startup accelerator for early-stage robotic companies laid the groundwork for what was to become ROS. With the main advantage of making robotic software modular and interchangeable, it applied the principles of open source collaboration to robotic software, reducing startup times for new systems drastically. This obviously results in ROS being able to do essentially everything, with the right combinations of community contributed ROS packages and self written code to combine everything into a working piece of software, but none of it perfectly well.

Still ROS, and by now ROS2 has become the defacto standard in academia and early stage robotic companies and it enjoys some adoption in industry as well. Especially in the field of robotic arms, where real time requirements are not as hard as with bipedal underactuated system, ROS2 prevails and is the dominant software framework for robotic software development. 

#### Core principles
Robotic software must adapt to the robot that it is applied on. Therefore ROS is set up to mirror the way robots are build, a collection of many individual parts that work together and depend on one another. For example, in a physical robot you might have motor controllers, batteries, a set of motion sensors and a camera. Each one of these components requires some software to control it. 

- The motor controller has to be controlled by the appropriate driver software that allows for communication via the bus system. Additionally we require some control algorithms to enable a simple control strategy that converts position data of the motor to individual commands over some period of time. 

- The batteries requires the same driver, but additionally there may be a load balancing model that has to be computed. Also battery charge estimation is to be performed by using sensor values measured within the battery. 

- Motion sensors have to have sensor fusion algorithms applied to itself, to make sure that the values that we measured are robust and correct

- The camera has to have computer vision techniques used on its images. This way object detection can be implemented, and computer vision models can be applied to the camera feeds for online evaluation of the surroundings of the robot. 

When building robotic software, we therefore want to follow this physical principle of modularization. A robot is a collection of different components that are connect together by the means of a communication network, mechanical connections and an electrical distribution network - our robotic software is a collection of different software processes that are connected together using a common communication standard on a communication network. 
