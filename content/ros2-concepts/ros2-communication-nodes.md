+++
title = "Communication between nodes"
weight = 6
+++

To make sure that data that is gathered, for example, on the camera is being transmitted to the actuator so that the robot would physically react to an obstacle that is in front of him, ROS2 provides a **communication system** so that different nodes can communicate with another. This communication system, also sometimes referred to as a data-distribution layer, will send information between different processes at runtime. The theory and software that underpins this functionality is not unique to ROS2 and is used extensively throughout other software projects in the industry.

#### Theory and structure of remote procedure calling
The goal of the ROS2 communication framework is to allow two nodes, that run at the same time to exchange information between them. Traditionally, when we think about programms, they are encapsulated from the rest of the operating system, running subsequently and are being executed after one another. However, as we discussed before, in robotics programming we structure our software into several small nodes that all run at the same time, similar to the microservice architecture employed by large scale software systems. 

The tool we use to make this work is called A *DDS*, ie.e. a Data Distribution Layer. We can think of this as a middleware, not quite on the level of the operating system but also not application level code. This middleware is not exclusive to ROS2, instead its being used all over the software landscape in many projects, a lot of them having nothing to do with robotics. A DDS has to do three main different types of work:

- **Discovery:** Discovering other instances of a DDS to be able initialize a connection
- **Data exchange:** Sending and receiving messages
- **Quality of service (QoS):** Controlling the reliability and latency of messages to meet deadline in cases where this is required. 

Most DDS frameworks are built on top of the principle of the publish subscribe architecture. The "source" of an information, i.e. some node for example, publishes the information that is to be transmitted on some channel. The recieving party, i.e. some other node is then subscribing to that communication channel to get any bit of information that is send. We can think of different structures, there may be only one publisher and many subscribers, a type of one-to-many relationship, where no bidirectional communication exists, so the publisher is not a subscriber at the same time. This structure would be called "Broadcast". There may also be bilateral communication channels where there are only two nodes connected, and each of them in publisher and subscriber at the same time.

We also refer to the implementation of such communication systems as remote procedure calling

#### The ROS2-DDS
We mentioned before that there are several implementations of a DDS. ROS2 gives us the opportunity to select a DDS among different options, with each of the different options suiting different needs. By default ROS2 supports [Eclipse **Cyclone DDS**](https://docs.ros.org/en/iron/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html), [eProsima **Fast DDS**](https://docs.ros.org/en/iron/Installation/DDS-Implementations/Working-with-eProsima-Fast-DDS.html) and [GurumNetworks **GurumDDS**](https://docs.ros.org/en/iron/Installation/DDS-Implementations/Working-with-GurumNetworks-GurumDDS.html). By default, unless specified otherwise, Fast DDS is used, so that's what we are going to talk about in this tutorial. With the exception of more specialized applications Fast DDS is almost always sufficient for most usecases, and there is little need to adapt. One notable exception to that however are real time compliant ROS2 systems. If real time compliance is a hard requirement, ROS2 has to be rebuilt from source with the **Connext DDS**, which is not included by default. Using Connext for other applications is not advisable, as the way it is to be run and used differs quite a bit from the other DDS systems, due to the inherent real time specifications. 

The communication with the DDS happens via the ROS2 client libraries, so it is very well possible to abstract away the details of the data distribution layer when building application level software. From the developer the interface of the data channel has to be defined and, most importantly, the type of data that is to be transmitted that has to be written down in a defined message file.

#### Defining data that is to be transmitted
When we want to transmit data we first have to define the shape and size of the dataset that we want to transmit. There are some defaults that are provided by ROS2 and dont require any more effort. These include the following data types:


| Name of type | C++ data type | Python data type | DDS data type |
| ------------- | ------------- | ---------------- | -------------- |
| bool | bool | builtins.bool | boolean |
| byte | uint8_t | builtins.bytes* | octet |
| char | char | builtins.str* | char |
| float32 | float | builtins.float* | float |
| float64 | double | builtins.float* | double |
| int8 | int8_t | builtins.int* | octet |
| uint8 | uint8_t | builtins.int* | octet |
| int16 | int16_t | builtins.int* | short |
| uint16 | uint16_t | builtins.int* | unsigned short |
| int32 | int32_t | builtins.int* | long |
| uint32 | uint32_t | builtins.int* | unsigned long |
| int64 | int64_t | builtins.int* | long long |
| uint64 | uint64_t | builtins.int* | unsigned long long |
| string | std::string | builtins.str | string |
| wstring | std::u16string | builtins.str | wstring |

It is also possible to combine these into arrays using the following avaliable structures

| Name of type | C++ data type | Python data type | DDS data type |
| ------------- | ------------- | ---------------- | -------------- |
| static array | std::array<T, N> | builtins.list* | T[N] |
| unbounded dynamic array | std::vector | builtins.list | sequence |
| bounded dynamic array | custom_class<T, N> | builtins.list* | sequence<T, N> |
| bounded string | std::string | builtins.str* | string |

Sometimes these datatypes are not enough and a dataset containing more than one value is required that can not simply be combined using one of the aforementioned structures. In this case we have to make a custom data structure that can be a combination of any of the previous datatypes and is sent in one go, therefore we dont have to wait for some parts of the message to arrive. We will now define an example of such a message definition. 

Defining custom messages should, by convention, happen in a seperate package whose contents are then imported into other packages. It is important to know that custom messages can only be defined in `ament_cmake` packages. As the packes themself dont contain any application level code, this has few consequences on the development experience. Obviosuly its also possible to import the message defined in a C++ package in a python package. To create such a package move back into your code editor, and open up the directory `ch2-2-defining-messages` that is present in the repository you've forked. 

Once inside that folder we can create new package on the `ament_cmake` build system with the name "example_messages". 

```bash
ros2 pkg create --build-type ament_cmake example_messages
```

Navigate inside that package. Here we will define the different message types. In general we have to distinguish between two different types of messages, the `.msg` pure message and the `.srv` message. Both serve the purpose of transferring data but are slightly different. Therefore we want to define two directorys inside the package, `msg` and `srv` isnide the `src` directory.

```bash
cd src
mkdir msg srv
```

All definitions of data transfer objects are made in files with either the ending `.msg` for messages and `.srv` for service messages. Essentially, `.msg` files are for unidirectional data transfer, where one node sends data and does not care about neither the recipient nor the response. The `.srv` files are for bidirectional data exchange between two nodes. We will talk about this topic in more detail in the following paragraf after finising the definition of the message package. 

We will create a simple `.msg` example, that will allow a node to transmit two *int64* type data points. Navigate inside the `.msg` directory and create a new file called `ExampleNumbers.msg`. Now open up this file and add the following lines:

```python
int64 ExampleNumberOne
int64 ExampleNumberTwo
```

In doing so we defined the message type *ExampleNumbers* that we can import into other nodes in the future. This message type now has two properties *ExampleNumberOne* and *ExampleNumberTwo*, both of them of type int64. Using this message we can now broadcast two 64bit Integers! Now we want to also create a service message file. Move out of the `msg` directory, and open the `srv` directory. Once there, create a new file called `ExchangeNumber.srv`. As mentioned before, these messages can implement bidirectional communication, meaning that the node that initiates the transmission will also recieve a response. This behaviour is directly mapped to the structure of the file structure of the message. Create this in `ExchangeNumber.srv`:

```python
int64 NumberToSend
---
int64 NumberToRecieve
```

Here we will transmit the `NumberToSend` property to some other node, and this node will return the `NumberToRecieve` property after doing some computation within the node. 

To be able to use these newly defined message types within other packages, we need to perform some housekeeping and add boilerplate code to the package definition. We have to add some additional build instructions to thr `CMakeLists.txt` file. 

```txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExampleNumbers.msg"
  "srv/ExchangeNumber.srv"
 )
```

Additionally there have to be edits to the `package.xml` file, to make sure that the `rosidl_default_generators` component of ROS2 is loaded properly, as it is required for building the message package. 

```xml
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

Now the package is ready to be built. As we may use this later on in a large project, we will only built this package. To do this using `colcon` execute the following command at the root of the project, e.g. *ch2-2-defining-messages*.

```bash
colcon build --packages-select example_messages
```

This sums up the build process of a simple package that contains a `.srv` and a `.msg` file. We will now take a look at how these can be used, and what type of data exchange can be performed using these. 

#### Types of communication channels
There are different ways how ROS2 Nodes can communicate with another. In general, this tutorial only talks about the communication inbetween nodes, within one node regular processes of Python or C++ can be used, e.g. encapsulating functionality in functions, etc. We may either *broadcast* information or *transmit* it. In ROS2 broadcasting happens using a Topic, transmitting information is done via Actions or Services. 

##### Topics
Topics can be compared to a radio frequency in FM/AM radio. There is no information available to the sender that specifies the number of receivers or if there even are any. There is also no direct response, the reciever may choose to answer but might also refrain from doing so. Multiple parties can speak on the same topic, but not at the same time. Topics are best used for continous data streams and are often periodically updated. For example, there might be a temperature sensor in our battery that is transmitting the current temperature every ten seconds. There can both be many senders on the same topics as there can be many recievers. There may also be many senders but only one receiver. 

##### Actions
Actions are designed for longer-running tasks that may require feedback, monitoring, or preemption. Unlike services, actions allow for continuous communication between the client and server while a goal is being executed, enabling progress updates and cancellation if needed. They are suitable for processes that take significant time or involve multiple steps, such as computing a new movement strategy or navigating to a waypoint. Actions provide structured feedback and result messages, making them ideal for tasks that must be interruptible and observable throughout their execution.

##### Services
Services are best used for quick, one-time interactions that complete rapidly and provide an immediate response. They operate in a request–response manner, where a client sends a request and waits for a server to process and return a result. Services are ideal for short, well-defined tasks such as retrieving system information, setting parameters, or performing brief computations. They should not be used for long-running operations or processes that require continuous feedback, as they are designed for fast, blocking calls that complete in a single exchange.

#### Making two nodes talk to one another
We will now try to get started with making two ros2 nodes talk to each other. To do so, open up the tutorial and navigate to the folder `ch2-3-make-the-nodes-talk`. Let us investigate it's contents. 

Inside the `src` folder can see four packages, `listener_node`, `talker_node`, `broadcast_node` and the `msg_package`. They are all defined as packages, so going further remember that the location where you can see the `src` directory, this is the root of the project and the place from which we want to execute the build commands. 

If you navigate to the `msg_package`, you can see that this is very similar to the message package we created in the last section. It contains two declerations of interfaces, a .msg file and a .srv file. Inside `Broadcast.msg` you can see that we will broadcast a simple single integer value.

```txt
int64 num
```

The service decleration `Transmit.srv` is a bit more complex, as it contains both a request and a response part.

```txt
int64 value
int64 value2
int64 value3
---
int64 result
```

These are the two types of information that we will send, once the `Broadcast.msg` as part of a topic and once the `Transmit.srv` as part of a service. Now take a look at the broadcast package, as this will demonstrate the implementation of a topic based communication channel. 

##### Using topics

Inside the `broadcast_node` you will find a package that is also called `broadcast_node`. Inside there are two files, one of them being called `broadcast.py`. Open up this file. This is an example of a simeple ROS2 naodes that implements a topic publisher. The communication that originates from this node is unidirectional, meaning that there is no response expected from any other node and the topic is simply broadcasted out. It is very well possible that a topic is broadcasted but no other node is listening to it. 

In the beginnign we are importing the `rclpy` library, that contains the ROS2 client library for Python. Additionally we are importing the `Node` class, as we will create a node that will broadcast information. Finally we are importing the `Broadcast` message type from the `msg_package` that we defined earlier. 

```python
import rclpy
from rclpy.node import Node

from msg_package.msg import Broadcast
```

The node itself, as explained before, is defined as a class that inherits from the `Node` class. In the constructor of this class we are initializing the node with the name `publish_information`. Up until this point, this is a regular implementation of a ROS2 node. In the next line we are defining a publisher. It is important to note that **publishers and subscribers are referring to topics** and not to services. 

```python
self.publisher_ = self.create_publisher(Broadcast, 'topic', 10)
```

The publisher is defined as an attribute of the node itself and consists of three properties. First we define the type of message that we want to send over the topic. This is what we defined previously with the custom defined message within the package `msg_package`. Next, we define the name of the topic, in this case it is simply named *topic*, but we are free to choose almost any other name. Finally, we define the size of the message queue. This is important as messages may be sent faster than they are being received. In most cases however, this is not strictly required and we will not go into more detail in the basic chapter of this tutorial series. 

In the following line we are introducing another concept od ROS2, the timer! A timer executes a certain function at a defined interval. In this case we are defining a timer that will execute the `timer_callback` function every half second, i.e. every 500 milliseconds. After this interval the methiod `timer_callback` will be executed.

```python
timer_period = 0.5  
self.timer = self.create_timer(timer_period, self.timer_callback)
```

Afterwwards we initialize the counter property of the node *i* as 0. This will be used to increment the value that is being broadcasted. The relevant properties of the publishing procedure happen within the `timer_callback` function. In here we define the message to be of type Broadcast, the message type we defined earlier. Then we set the property `num` of the message to be equal to the counter property *i*. After this we log the information that is being broadcasted, so that we can see it in the terminal. Finally we publish the message using the publisher that we defined earlier. 

```python
msg = Broadcast()
msg.num = self.i
self.publisher_.publish(msg)
self.get_logger().info('Publishing: "%s"' % msg.data)
self.i += 1
```

The main function is quite similar to the ones we have set up before, we initialize `rclpy`, create an instance of the `ExamplePublisher` node and spin it up to have the code working. Finally we destroy the node and shutdown `rclpy` after we are done. 

```python
rclpy.init(args=args)

minimal_publisher = ExamplePublisher()

rclpy.spin(minimal_publisher)

minimal_publisher.destroy_node()
rclpy.shutdown()
```

##### Using services

We can also transmit information using services. The advantages of services is, that they allow bidirectional communication, meaning that the node that is sending the information can also recieve a response from the node that is processing the information. To demonstrate this, we will take a look at the `talker_node` package. Inside this package there is a file called `service_server.py`. Open up this package directory, you will find a file inside the `src` subdirectory called `server_node.cpp`. This file contains the node that is serving new service messages. 

```cpp
#include "rclcpp/rclcpp.hpp"
#include "msg_package/srv/transmit.hpp"
#include <memory>
#include <inttypes.h>
```

In the beginnign we import the usual packages, in this case the `rclcpp` package for C++ and the custom defined service message from the `msg_package` package. The `memory` and `inittypes` packages are standard C++ packages that are required for memory managment and integer type definitions. In case we define our functiosn differently, we could get around using them. 

```cpp
void handle_transmit(
  const std::shared_ptr<msg_package::srv::Transmit::Request> request,
  std::shared_ptr<msg_package::srv::Transmit::Response> response)
{
  response->result = request->value + request->value2 + request->value3;
  RCLCPP_INFO(
    rclcpp::get_logger("transmit_server"),
    "Incoming request:\n value: %" PRId64 "  value2: %" PRId64 "  value3: %" PRId64,
    request->value, request->value2, request->value3);
  RCLCPP_INFO(
    rclcpp::get_logger("transmit_server"),
    "sending back response: [%" PRId64 "]", response->result);
}
```

This is the initial function that will handle incoming service requests. It takes two arguments, the request and the response. The request is a shared pointer to the request part of the service message, the response is a shared pointer to the response part of the service message. Inside this function we are simply summing up the three values that are being sent as part of the request and setting this as the result property of the response. Additionally we are logging both the incoming request as well as the outgoing response to the terminal. 

```cpp
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("transmit_server");
  auto service = node->create_service<msg_package::srv::Transmit>("transmit", &handle_transmit);
  RCLCPP_INFO(rclcpp::get_logger("transmit_server"), "Ready to sum three int64s.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

The main function is similar to the ones we have seen before. We initialize `rclcpp`, create a node with the name `transmit_server` and then create a service on that node. The service is of type `Transmit`, the custom defined service message we created earlier. The name of the service is `transmit` and the function that will handle incoming requests is `handle_transmit`, the function we defined previously. After logging that the server is ready, we spin the node to have it process incoming requests. Finally we shutdown `rclcpp` when we are done.

As we are now defining a service, we require a secondary node that will send requests to this service and process the responses. This is done in the `service_client.py` file within the `listener_node` package. Open up the package. In there you will notice two files, `service_client.py` and `listener.py`. We will focus on the `service_client.py` file for now, as this is the node that will send requests to the service server we defined earlier. 

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from msg_package.srv import Transmit
```

Here we are importing the required packages, similar to what we have done before. The only new import is the `sys` package, which is a standard Python package that allows us to access command line arguments. 

```python
class TransmitClient(Node):
    def __init__(self):
        super().__init__('transmit_client')
        self.cli = self.create_client(Transmit, 'transmit')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        self.req = Transmit.Request()

    def send_request(self, a, b, c):
        self.req.value = int(a)
        self.req.value2 = int(b)
        self.req.value3 = int(c)
        return self.cli.call_async(self.req)
```

This is the definition of the `TransmitClient` node. In the constructor we initialize the node with the name `transmit_client`. Then we create a client that will connect to the service named `transmit`, which is the service we defined in the service server node earlier. After this we wait for the service to become available, as it may take some time for the server to start up. Finally we create a request object that will be used to send requests to the server.

```python
def main():
    rclpy.init()
    node = TransmitClient()

    try:
        for i in range(5): 
            a, b, c = i, i + 1, i + 2
            future = node.send_request(a, b, c)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                node.get_logger().info(f'[{i+1}] result: {future.result().result}')
            else:
                node.get_logger().error('service call failed')
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
```

The main function is similar to what we have seen before. We initialize `rclpy`, check if the correct number of command line arguments are provided, and create an instance of the `TransmitClient` node. Then we send a request to the server using the command line arguments as values. We wait for the response using `spin_until_future_complete`. If we receive a result, we log it repeatedly to the terminal. Finally, we destroy the node and shutdown `rclpy`.

##### Launching the nodes
Now we want to launch these nodes locally, to see how the information is transmitted between the different nodes. To do so, open up a terminal and navigate to the root of the project, e.g. `ch2-3-make-the-nodes-talk`. First, we will build all packages using `colcon`.

```bash
colcon build --symlink-install
```

In case we have previously build the project already and made changes, its good practice to clean the build, install and log directories before building again.

```bash
rm -rf build install log
colcon build --symlink-install
``` 

Then we fill first try out the topic based communication. Open up an additional terminal window. In both windows source the install setup file.

```bash
source install/setup.bash
```

Now the build project is intialized and ready to go. In the first terminal window, we will launch the broadcast node that will publish information on a topic. To do so, execute the following command:

```bash
ros2 run broadcast_node broadcast
```

Then, in the second tutorial, execute the listener node that will subscribe to the topic and receive the information.

```bash
ros2 run listener_node listener
```

You should now see in the terminal window of the listener node that it is receiving the information that is being broadcasted by the broadcast node! Now let us try the same for the service node. Stop both processing by hitting `CTRL+C` in both terminal windows. Then, in the first terminal window, launch the listener node with the three number 1, 2 and 3.

```bash
ros2 run listener_node service_client 1 2 3
```

Then, in the second terminal window, launch the talker node that will serve the service requests.

```bash
ros2 run talker_node server_node
```

You should now see in the terminal window of the service client that it is receiving the summed up result from the service server! Congratulations, you have successfully made two ROS2 nodes talk to each other using both topics and services!