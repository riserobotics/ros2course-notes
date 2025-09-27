+++
title = "Nodes & Packages"
weight = 3
+++

#### The ROS2 node concept
We will now apply these concepts to the purpose of robotic software. The equivalent to a mechanical device, like an actuator or joint bearing is called a **node**. You can think of a node as an individual process that is controlling some thing on your robot. For example, if you have a physical camera mounted to the chassis of a small quadruped, the software controlling the aperture control of that camera would be written as a node. If you have an actuator that is to be controlled using a PID-control loop, the code implementing this PID loop would be written as a node.

Obviously a robot is not just one single camera, or a single actuator. A real robotic system would be comprised of many different nodes, that each serve a very different task. Also, there may be multiple nodes that can be attributed to a common purpose, i.e. there might be a node housing the camera driver and another node housing a computer vision algorithm. Together they allow us to use the hardware capabilities of the physical device in the software system we are building. To order this, ROS allows us to put multiple nodes together into a folder so that they form a **package**. As there may be many different people using the particular camera, we can share these packages on a packet exchange called `rosdep`, similarly to how pypi for python packages and apt for debian applications works. We will learn more about this in [Chapter V](/ros2-ecosystem/).

![Image 1: Schematic view of the ROS2 node setup](/images/ch2-nodes-overview.png)

We will now set up a ROS project to demonstrate how nodes and packages can be created an used. 

#### Setting up the Tutorial environment
Make sure that you have the ROS container setup like described in the [tutorial](/installation-and-setup/installation/) on installing the setup. From now on, we assume that you have the Docker container installed and feel comfortable using it. From this point onwards we will also assume that you execute all code **within** that Docker container and not on the host system. To interact with the container easier, you can open up VScode on the host computer and click on the blue button in the bottom left hand corner. In the resulting drop-down menu, select "Attach to a running container". In the following menu select the container you just started. Now, your VScode interface opens up the files in the container, makes the internal terminal run within the container and allows you to execute files in the container.

{{% notice info %}}
During this course want to use the `git` version control system, to track changes and progress, and also allow feedback to be given on your work. Therefore, before cloning the repository, fork it to your own account using the following steps. 
Go to GitHub and navigate to the repository, which can be found [here](https://github.com/riserobotics/ros2course). On the top right part of the window, you will find the Button "Fork". After you press that button, a context menu will open. make sure that the dropdown menu under "Owner" is set to your private GitHub account, i.e. the account that has your account name *not* "riserobotics". The repository name will be pre set to "ros2course", we recommend keeping it this way. The same goes for the description, there is no real need to change it. Before clicking on "Create fork" make sure to uncheck the checkbox "Copy the main branch only", to make sure that all branches of the repository are forked. 
If you want to learn more about what forking means, see [here](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) for some information.
{{% /notice %}}

To begin, jump into the docker container and open up an internal terminal. Then go ahead and clone the forked repository to the docker container. If you choose your own name for the repository, make sure to adjust the link accordingly. 

```bash
git clone https://github.com/[Insert your  GitHub username here]/ros2course
```

{{% notice tip %}}
Depending on jow you set up your GitHub, you may face difficulty authenticating to the service. If you set up `ssh` based authentification, either import your keys locally to the Docker container which is not recommended as you may share the container with someone and then unwillingly give them access to your keys, or you mount the key as a local volume and give the container access to said volume. For most people however, you will authenticate with `https` based procedures. To do this, you can either install the github command line utility `gh`, or create a token from the Developer settings of your account, as password based authentication has been deprecated. You can find tutorials on token based cloning [here](https://graphite.dev/guides/git-clone-with-token), and on setting up the github command line utility [here](https://cli.github.com/manual/gh_auth_login).
{{% /notice %}}

{{% notice tip %}}
If you are trying to install `gh`, know that the Docker container does not have the `apt` remote repositorie index lists locally, so you will have to run `sudo apt update` before being able to install anything.
{{% /notice %}}

Now move inside of the cloned repository using the `cd` tool

```bash
cd ros2course
```

If you type `ls` to see the contents of the directory, you can see that they are ordered by the different chapters of this tutorial series. As we are in the second chapter, and are working on the first task, move to the directory `ch2-1-creating-nodes`. Except for a `README.md` and a subdirectory `tests`, this should be an empty directory. Here, we want to set up our first ROS project and create some nodes!

#### Setting up the ROS2 workspace
Every ROS2 project has to run within a ROS2 workspace. At its core the ROS2 workspace is just a folder containing one or more ROS2 packages. Usually we want to create a new folder that will contain this workspace, we are in theory free to choose the name, but it is usually helpful to indicate the purpose of the directory in the name. Create a folder called `ros2-node-test-workspace` and a folder `src` within it. Afterwards, move into this folder.

```bash
mkdir -p ~/ros2-node-test-workspace/src
cd ros2-node-test-workspace
```

The `src` directory will serve as the home for all our ROS2 packages that we are going to build. Again, in theory we ware not required to put them into the `src` directory, it is however best practice to do so, making the overall repository structure cleaner and more readable. Up until this point we have not made anything that requires ROS2 yet, all of the tools till now are standard Linux command line tools.

#### Creating a ROS2 package
Our ROS2 workspace is currently empty, lets change that! We can build a package in two ways, either by putting together the required files in a specific structure ourself, or by using the `ros2` command line utility. Here, we will choose the latter option.

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 ros2-sample-package-python
```

This command creates the structure of a minimal implementation of an empty package for us, at the point where we ran the command. We define the license as the `Apache-2.0` license, and we name the package `ros2-sample-package`. Instead of `ros2-sample-package` we could have also chosen any other name, that is made up of utf8 characters. `--build-type` defines the build system that we use for this package. We will get into this later in more detail, for now just note that this sets the programming language that you will use in the package. `ament_python` will result in this being a python package, `ament_cmake` results in a C/C++ package. We will create both as an example, so also execute:

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 ros2-sample-package-cmake
```

To check if everything worked as expected, you can run a test using this command. As an alternative, you can see in the tab menu how the directory structure should look:

{{%expand "Correct directory structure at this point" %}}
```bash
.
├── ros2-sample-package-cmake
│   ├── include
│   │   └── ros2-sample-package-cmake
│   └── src
├── ros2-sample-package-python
│   ├── resource
│   ├── ros2-sample-package-python
│   │   └── __pycache__
│   └── test
│       └── __pycache__
└── tests
    └── __pycache__
```
{{% /expand%}}


```bash
pytest tests/test_structure_correct.py -v
```

It is important that we should not create packages within other packages, as this will potentially lead to some problems. It is best practice, to make every package at the root of the `src` directory. See the following for an example of how **not** to do it.

```bash
.
├── ros2-sample-package-python
│   ├── resource
│   ├── ros2-sample-package-cmake
│   │   ├── include
│   │   │   └── ros2-sample-package-cmake
│   │   └── src
│   ├── ros2-sample-package-python
│   │   └── __pycache__
│   └── test
│       └── __pycache__
└── tests
    └── __pycache__

```

We have now created two ROS2 packages, one of them is called `ros2-sample-package-cmake` and the other `ros2-sample-package-python`. Now we will proceed with taking a look at what these packages are composed of. 

{{< tabs >}}
{{% tab name="Python (ament_python)" %}}
- Within the folder resources, there is a empty file called exactly like the package it is build in. Even tough this file is empty, it's quite important and cant be deleted! When running, ROS2 needs to check what packages are installed, this file serves as the "marker" file that ROS2 can search for to see that the package is installed. It's empty, and will almost alwyas stay empty, but it is there for a reason and cant be removed. 
- The folder with the same name as the package contains the `__init__.py` file, that let's the python interpreter know that this is a package and it can be imported in a different context. This is the place were you will put the python files containing the actual application code. 
- Within `test`, there are some default test for checking flake8 compatibility, the linter that both we and the ROS2 project uses, as well as for the correct license formatting. You may delete these files, but putting tests in here is generally quite useful.
- The LICENSE file keeps the LICENSE for the package, that was set when creating it using the ROS2 command line utility.

The more interesting content can be found in the three files in the directory, `package.xml`, `setup.cfg` and `setup.py`. These contain the information that is actually relevant to the package and where a lot of the configuration of a package can happen. To understand them, be reminded that packages with ROS are very much meant to be shared across teams, labs and companies. Some of the information that is specified here, is only describing the packge to a registry that other people can then search, if you decide to upload your package. 

- `package.xml` contains all the information that you can find if you search the ROS2 package registry. The name that is specified here, should be the same as the name of the folder that the whole package is in. Th version can also be defined by the author of the package, same goes for description and the linked email address, used to follow up with the authors in case of problems. More interestingly, the section "build_depend" declares the dependencys that are required by the project. Be aware that this does not refer to Python packages or C++ header files, this references other ROS2 packages. In the case of our package, as we don't require another ROS2 package as a reference, this is empty for now. "test_depend" is a similar nominator, for describing the dependency on a standard ROS2 test command, or any other test package that is required for running unit tests used within the package you are describing. A powerful tool is the "export" keyword, you can use this as a container for exporting additional information, for example which type of build system is to be used. 
- ``
{{% /tab %}}
{{% tab name="C/C++ (ament_cmake)" %}}
You will have to use VIm inside of the container that you are running. Luckily it is already installed, so there is no configuration required! 
{{% /tab %}}
{{< /tabs >}}

#### The build system