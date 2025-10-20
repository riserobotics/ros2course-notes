+++
title = "Executors"
weight = 1
+++

Up until this point, we have not thought more about what actually happens when we call `rclpy.spin(node)` in our main program. In traditional simple python development, we are usually only aware of programs being executed *sequentially*, i.e. one bit after the other. As we are building robotic software, this would be unfortunate, as we have software architectures that require many processes to run at the same time and - most importantly, we have to be able to control how and when they run. At this point it makes sense to get to know the different executors that ROS2 offers a bit more. 

#### What is an executor
Executors are the components of ROS2 that are responsible for running the nodes that we build. They use one, or if required multiple, threads of the underlying operating system, e.g. mostly Ubuntu, to execute the ROS2 nodes. We can handle this from simple to complicated with a rising degree of control as we get towards the more complicated options.

Functions and patterns that are affected by the choice of Executors include Subscriptions to Topics, Timers that lauch reocurring functions, service servers that transmit services and action servers that transmit actions. 


#### Types of Executors

{{% notice warning %}}
Be aware that there have been quite substantial changes to the Executors between ROS1 and ROS2. In case you have already gained some experience with ROS1, make sure to study this topic again, as fundamental concepts have changed. 
{{% /notice %}}

1. The `spin` executor. This is the simplest executor that is avaliable through ROS2. Here, the node will be executed within the main thread, in practice `spin` refers to a simple, single threaded executor. In the core of ROS, when the `spin` executor is called, the middleware is being surveilled for incoming messages and request to the node. These incoming messages are stored within the middleware, it does not matter which one is selected here as all offer support for this behaviour, until the node is scheduled to take over within the main thread. Then the information is retrieved from the middleware and processed. The `spin` executor is easiest to use, as there are no behaviors that are unintuitive on first view. 

2. The `SingleThreadedExecutor`. This is quite similar in behavior to `spin` when configured right, but offers more options when it comes to customizability. We will make use of this executor quite often. You can think of the `SingleThreadedExecutor` as the explicit version of what `spin` does behind the scenes. Using this executor, you can have more control over when and which nodes are added, when the executor starts and how the shutdown of the nodes is handled.

#### The `SingleThreadedExecutor`
As an example for how the `SingleThreadedExecutor` and the regular `spin` differs, see the code examples in `ch3-1-executor-comparison`. You will find three options that all implement a similar node using a different executor. In `spin_one` we can observe the simple `rclpy.spin()` executor, abstracting the complexity of the execution of nodes away from the developer. In comparison to that, check the `exec_two_nodes` file, where we can see the `SingleThreadedExecutor`.  