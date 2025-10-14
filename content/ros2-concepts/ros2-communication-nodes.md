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