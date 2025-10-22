+++
title = "Installation"
weight = 2
+++

Great! With all formal details out of the way we can start to dive into ROS2. In this tutorial we will first install the framework an then start explaining how it works, to give you the opportunity to try everything on a practical example, while reading or watching the tutorial. Also, as you will notice later on, we will install ROS2 in a way that abstracts most of the "technical details" away for a later date. So, don't worry about not understanding anything for now, this is just a brief period and we will explain everything in much more detail as soon as we are done here. 

#### Installing Docker
We will install a ROS2 development environment using the Docker containerization technology. This allows us to not worry about the operating system layer just now, and also standardizes the development environment between team members. Additionally, in case something went majorly wrong, and you want a clean start - no need to setup your computer again, just start a second Docker Container. 

Docker is an open source based technology that sits somewhere in between a virtual machine and a pure application. The virtual machine simulates a complete computer, including (practically) running it's own OS Kernel. A container is basically a virtual machine, just that it doesn't implement all of it's OS features, but uses those of the host system. Therefore a container has most of the features of a virtual machine, but uses less resources at the same time.

The docker engine is the part of the program that allows the containers to run on your operating system. Installing this is somewhat easy on Linux, and pretty hard on Windows and MacOS. To make things easier we recommend you to install Docker Desktop, a graphical user interface for interacting with the docker engine, that handily also installs everything for the respective operating system on the host machine. The exact details of how to install this, differ by operating system and are documented on the website of the Docker project. Please follow the instruction there to install Docker Desktop. 

{{% button href="https://docs.docker.com/desktop/setup/install/linux/" %}}Install on Linux{{% /button %}}
{{% button href="https://docs.docker.com/desktop/setup/install/windows-install/" %}}Install on Windows{{% /button %}}
{{% button href="https://docs.docker.com/desktop/setup/install/mac-install/" %}}Install on MacOS{{% /button %}}

You should end up with the Docker Desktop program on your computer, that after starting looks something like this. To get the full DOcker Desktop feature set, getting a Docker account is useful, but there is technically no need to do this. 

![Image 1: Overview of the Docker Desktop GUI](/static/images/installation-and-setup-docker-desktop-screenshot.png)

#### Getting the RISE ROS2 Container
Now we have to get the container that we want to run using Docker. In our case this includes all parts of the operating system as well as ROS2 pre installed.  There are multiple ways on how we can install the container, depending on your operating system or of you prefer a graphical install using the Docker Desktop GUI. See below for a fitting tutorial.


If you are on a Linux based operating system, you can install this container using the `docker pull` command line utility. The same goes for Windows and MacOS using Powershell and the Terminal respectively. It should be installed automatically, as a result of you installing Docker Desktop. If not, go back and check your installation. In case of further problems, a workaround would be the sole installation of the docker CLI and the docker engine, for a tutorial on doing this see [here](https://docs.docker.com/engine/install/) (Linux only). 

To get the RISE ROS2 container, execute this command ina directory that you deem fitting. Be aware, that this will download around 1.9GB of data:

```bash
docker pull ghcr.io/riserobotics/rise-docker-ros2-container:main
```

#### Launching the RISE ROS2 Container

{{% notice tip %}}
You will have to do the following steps every time you shut-down/sleep your computer. If you are unsure if a container is still active, either take a look at the "Containers" menu in the Docker Desktop GUI in the left sidebar, or execute the following command to _stop all images_! Be sure that this is what you want to do! `docker stop [name of the docker container here]`. If you want to _remove_ all containers as well, run the same command with `docker rm [name of the docker container here]` attached to the back. 
{{% /notice %}}

Now it's time to launch the container we just downloaded. This can be done either via the command line, which is the preferred way, or using the Docker Desktop GUI if you feel more comfortable this way. 

{{< tabs >}}
{{% tab name="Command line interface" %}}
You can run the pulled image using the following command, executed in the same terminal as where you pulled the image!

```bash
docker run -d --name rise_container ghcr.io/riserobotics/rise-docker-ros2-container:main sleep infinity
```
The container is now running in the background, and you can jump into it using the following command:

```bash
docker exec -it rise_container bash
```

To get to home type the following:

```bash
cd ~
```

{{% /tab %}}
{{% tab name="Docker Desktop GUI" %}}
Running the container from the DOcker Desktop GUI is quite simple! Navigate to the "Images" menu in the left sidebar and find the container "ghcr.io/riserobotics/rise-docker-docs-container". Use the "Play" symbol at the end of line to start the container. In the following context menu, simply select "Run". 

![Image 2: Screenshot of the "Images" menu](/images/installation-and-setup-docker-desktop-images.png)

To get a terminal on the container, you can either navigate to the "Containers" menu in the same sidebar and find the one with the matching image name. Then at the very back, you will find three dots that when clicked on provide a context menu that offers the option "Open in Terminal". Alternatively you can also use the command line, simply type the following command in any terminal on your computer.

```bash
docker exec -it rise_container bash
```

{{% /tab %}}
{{< /tabs >}}


#### Installing a code editor

{{% notice tip %}}
You are free to use a different code editor, perhaps you already have your favorite editor installed anyways! However, be warned that we will only provide help if you use VSCode or Vim. If you want to use another editor, it is your responsibility to figure out how to connect containers, and make sure they work robustly.
{{% /notice %}}

In order to edit code, we want to use a Code Editor. We recommend VSCode if you want to use a graphical editor, or Vim if you prefer the terminal environment. 

{{< tabs >}}
{{% tab name="Install VSCode" %}}
VSCode is a graphical editor that is available for many operating systems. You can find the installation instructions for your respective system here:
- **[Linux](https://code.visualstudio.com/docs/setup/linux)**
- **[MacOS](https://code.visualstudio.com/docs/setup/mac)**
- **[Windows](https://code.visualstudio.com/docs/setup/windows)**

Make sure to perform this install on the host computer, _not_ inside of the container. 
{{% /tab %}}
{{% tab name="Install Vim" %}}
You will have to use VIm inside of the container that you are running. Luckily it is already installed, so there is no configuration required! 
{{% /tab %}}
{{< /tabs >}}



**Congratulations!** You have now set up the Docker container that we will use for this tutorial series and the development efforts in the RISE project. Proceed with the next chapter to get started with learning the basics of ROS2!
