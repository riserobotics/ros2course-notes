+++
title = "The colcon build tool"
weight = 5
+++

Within ROS2 we have to use the build system, to make the code that we wrote into our nodes usable and executable for the ROS2 core. Back in the day, in the times of ROS1, the build system for these purposes was called `catkin`, which is a heavily customized set of CMake based routines. This build system had a couple of drawbacks and problems that pushed the maintainers of the ROS project towards developing a new system for the release of ROS2. This new system is made up of two parts, the `ament` build system and the tooling that the developer uses to talk to this build system from the command line: `colcon`. The type of `ament` that is used depends on the type of package that is to be build. When we defined a package in the last chapter, you may remember that is twas important to specify this build system when setting up the package in the `package.xml` file.

```python
<export>
    <build_type>ament_python</build_type>
</export>
```

Ament itself is quite complicated to get to know, and not relevant for everyday development of robotic software - unless you want to dive deep into performance optimization on the compiler level. Thats why we wont make an effort to learn the details about this build system and instead assume it to work like the developers intended: as a build system for different modules ()"packages") together in one workspace that have to run on a robotic device. The more important part when it comes to actually developing software with ROS is the `colcon` build *tool*, i.e. the command line application that we use to interact with ament.  

#### Understanding colcon
Colcon is a build tool, it does not do the building itself but instructs `ament` how to do so. Technically, no one is stopping you from calling the `ament` build system directly, they are also installed on your computer if you installed ROS2 correctly. However, calling the build system directly is usually only an added hassle that does not provide much advantages to the user.

`Colcon` generally works on the principle of a "workspace" based build system, i.e. you dont have to manually type  out all the files that you watnt o build, and the specific oder and dependencies that they are to be build. Instead the workspace, i.e. a directory on your computer has to be structured ina specific way and `colcon` does the rest. The structure that is required for `colcon` is exactly the structure that we have set up in the previous chapters around packages, nodes and `package.xml` files. When you use colcon, it will search through all subdirectoys below the level where the command is invoked for packages. To make everything more legible and easier to understand its best to put all of your packages into the `src` subdirectory.

Only those folder around a `package.xml` file, colcon understands to be a package, and is able to build. Missing this file therefore results in the tool just skipping the set of files and not building them. 

Usually we often consider building to be closely linked to compiling something, however we also have to build packages written in interpreted languages such as Python. This obviously has nothing to do with compiling, but more with linking the intepreter to the location of the code files that it has to run when they are called up.

#### Using colcon
Actually using `colcon` is quite easy! There is no need to perform an installation, as the package is installed together with the regular ROS2 installation. Also, as for the purposes of this course we will only talk about the basics of `colcon`, digging deeper is usually only required for configuring custom build systems.

Let's build the package that we created during the last chapter of this tutorial series! First start the Docker container and attach your VSCode session to it. Then open up a Terminal that is linked to the container. In case you have questions regarding how to do this, see the [Installation](/installation-and-setup/installation) chapter of this series. Inside of this tutorial, navigate to the workspace, we called this workspace `ros2-node-test-workspace`. You should be able to call `ls` and see something similar:

```bash
root@2ffb27508d09:~/ros2course/ch2-1-creating-nodes/ros2-node-test-workspace# ls -l
total 4
drwxr-xr-x 5 root root 4096 Oct  1 09:59 src
```

This place, where we can see the `src` directory, is called the *workspace root*, not because we are the user root, but because it's the root point of the workspace. If we want to build all packages in the subdirectiy `src` and not have any other special requirements that differ from the defaults we can simply call the build command.

```bash
colcon build
```

This will result in the following output.

```bash
Starting >>> ros2-sample-package-cmake
Starting >>> ros2-sample-package-python
Finished <<< ros2-sample-package-python [0.92s]                                                               
Finished <<< ros2-sample-package-cmake [0.96s]

Summary: 2 packages finished [1.09s]
```

You have now build the packages that are located in the `src` folder! IF you run `ls- l` again, you will however see that there are new directorys in the workspace root. 

##### log
Within this folder you will find the logs that concern the build process. These are not the logs that come up while you run the programm, those are by default routed to the terminal and have to be stored seperatly if you wish to do so. Inside the folder is another directory that will be called `build_[Date and time when building]`. Inside this package we have the detailed logs of the build process, where you can investigate a potential error that would come up during building. If you are interested, this is also an interesting place to understand the anatomy of the build system, as the CMake interface is quite exposed here. The unsorted but complete log of the build process can be found in the file `logger_all.log`.

##### build
When colcon is building a package, it keeps some build artifacts when finishing to make subsequent build runs faster. The build directory is where these are stored. They are seperated byt the name of the package and are overwritten automatically in the next build if something changes. It is best to not tamper with these artifacts, as this could break the results of the coming builds. Also, in case your build seems to be stuck on a problem you are sure you've fixed, it is useful to delete this directory. colcon will recreate it the next time you run `colcon build`, and no old artifacts that are erroneously kept can corrupt your build result.


##### install
This is the main result of the colcon build process, as it stores the result and the scripts that you use to source these results. Inside this folder you will find seperate directorys for all the packages that you've defined in your workspace, where the resulting files are kept. If you compare the python package to the cmake package, you will see the difference in structure and amount of packages. Next to the directorys that hosue the names of the packages, youll also find several scripts and shell files. These are required so that you can source the result of the build process to your current shell for execution. Currently colcon supports mainly the `bash` and `zsh` shell on Linux and MacOD, and `powershell` on Windows. To source the result of the installation, wait for the build process to finish and then, from the root of your workspace, invoke the following command (for bash, use .zsh for the zsh shell).

```bash
source install/setup.sh
```
{{% notice warning tip %}}
Especially if you are making changes to the ROS2 codebase and aim to execute the code on the robot, make sure to double check the results of your build process,a nd wether or not you are sourcing the correct `setup.sh` file
{{% /notice %}}

This is really important. If you dont call this command, the changes you made to the ROS2 code will not apply and you cant use them. Now the results of your code have been built and are ready to be executed on the robot. Before doing so, we will however take a look at how the two nodes we just build, can talk to another in the [next](/ros2-concepts/ros2-communication-nodes) chapter. 
