+++
title = "Prerequisites"
weight = 1
+++


#### Required knowledge

We don't require previous knowledge in robotic programming or ROS1/2 for following along with this course. At the same time, getting up to speed with ROS2 and keeping track with this tutorial is much harder if you don't have at least some of the following prerequisites. 

- You should have basic familiarity with **Linux**. ROS2 as a framework, and the RISE exoskeleton in particular, run a GNU/Linux distribution as an operating system. We will use a containerization technology (don't worry about what that is, understand it is not a prerequisite!) to abstract that away from you as much as possible, and in many cases there will probably be a way to avoid directly using Linux, however using it and knowing the basic workings makes many things a lot easier!

- You should feel **comfortable using the command line**. Developing ROS2 systems happens with (almost) no graphical tools, therefore you should feel comfortable navigating a computer using a Unix like shell. It would be good if you know, for example: how to move between directories, execute and stop programs and processes, install external software packages and use command line interface tools. All tutorials and code examples here assume a bash shell. There is no need to be proficient in shell scripting, know detailed debugging toolchains or go below the level of the shell.

- You should know some **common software development tools**, that are used to develop code together in a team and make sure that quality standards are upheld. Especially, you should know the basic principles behind the version control system git. We will use git in combination with the remote source code host ["GitHub"](https://en.wikipedia.org/wiki/GitHub). Ideally you have an idea what add, commit, push, branch, checkout, fetch and pull mean in the command line, and are able to navigate the GitHub web interface to create pull requests, issues and branches. In case you feel unsure about your capabilities here, git is quite easy to learn and can be picked up quickly, see some tutorials [here (Guide from RedHat)](https://developers.redhat.com/articles/2023/08/02/beginners-guide-git-version-control) and [here (YouTube)](https://youtu.be/HkdAHXoRtos?si=MB2Z33IzkqEiz-qd).

- At least basic to good command of one of the two programming languages used in this project: **Python** and **C/C++**. Yes, there is technically the option to write ROS2 code in many more than the two languages described here. However, mixing and matching many different languages together makes things generally harder to maintain and harder to understand. Unless there is a _very_ good reason to do so, we will not do so. Therefore you should feel productive in at least one of the two languages. Making practical use of a language in a specific setting is a great opportunity to learn and acquire more proficiency, not starting from zero is important here! Together with the language itself, it's also helpful to have a general understanding of the toolchains associated with each language, as well as the surrounding ecosystem. 


While we obviously provide help and support if during the course of the semester you have questions regarding one or more of these topics, knowing and being able to use them to at least some basic, productive degree is your responsibility. 

#### Useful knowledge

{{% notice note %}}
The topics described here are "nice to have" but not required, and it's perfectly fine to start the tutorial without them. Dont feel intimidated if you have never heard of them, they are very much learnable on the go!
{{% /notice %}}

There are several topics that are very useful to know when developing code for the RISE exoskeleton. These include but are not limited to:

- Expertise with the **EtherCAT** bus system and communication standard, that we use to make real time control of actuators possible and read back sensor values from different corners of the system. To do this, we use the [SOEM](https://github.com/OpenEtherCATsociety/SOEM) driver setup.  

- Knowledge of **Real time operating systems** for x86 architecture platforms. We will have to make some parts of the overall software stack compatible to some real time requirements at some point in the future. 

- Knowledge of **relevant standards** for the development of software for mechatronic systems in the medical device space. We dont aspire to become a medical device anytime soon, but still try to align our development process as best as possible. 


#### Hardware requirements

{{% notice warning %}}
In case of running Linux, we will help with operating system related problems only if you are running a "mainstream" distribution. This includes Ubuntu, Debian, Fedora or Mint, as well as close derivatives of those. If you happen to have a problem on your custom NixOS installation, if your Gentoo based machine can't compile the code, because you implemented a custom fork of the Kernel, or any similar problem on a more "exotic" distribution occurs, we assume that you have enough technical proficiency to solve the problem yourself and will not provide help!
{{% /notice %}}

To follow along with this tutorial as well as the eventual software development there are some hardware requirements regarding your computer. Ideally you have a **x86** based system, running either Linux (preferred), MacOS or Windows.  Depending on your operating system there are specific requirements that have to be met:


{{< tabs >}}
{{% tab name="Linux" %}}
- 64-bit kernal and a CPU that supports virtualization. Check the documentation fo your CPU manufacturer for details. We require support for KVM, you may have to enable this in your computers BIOS settings. 
- QEMU version 5.2 or later. The latest version should be fine. 
- A distribution that is based on the systemd init system
- Either the GNOME, MATE or KDE desktop environment
- At least 4GB of RAM, 8GB is much preferred

We make use of containerization technologies, running containers in a nested virtualization scenarios, i.e. running a VM in another VM, might cause problems, therefore it's best to run your operating system natively on your computer. 

{{% /tab %}}
{{% tab name="MacOS" %}}
- The most current or one of the two previous MacOS release. Currently you are safe with "Sequoia" and "Sonoma".
- At least 4GB of RAM.
- If you have a computer with the M1 chipset, make sure that the Rosetta translation layer is turned on by executing the following command in the terminal. 

```bash
softwareupdate --install-rosetta
```

{{% /tab %}}
{{% tab name="Windows" %}}
- Windows 11 64-bit in either the Enterprise, Pro or Education version 22H2 or higher
- Windows 10 64-bit with either the Enterprise, Pro or Education Version 22H2 or higher. 
- Having Hyper-V enabled for virtualization. You may also have to enable Virtualization in the terminal
- Al least 4GB of RAM.

Alternatively, if you have WSL enabled:
- WSL version 2.5 or higher
- Windows 11 64-bit: Home or Pro version 22H2 or higher, or Enterprise or Education version 22H2 or higher.
- Windows 10 64-bit: Minimum required is Home or Pro 22H2 (build 19045) or higher, or Enterprise or Education 22H2 (build 19045) or higher.
- The same hardware requirements as above. 

{{% /tab %}}
{{< /tabs >}}

You also should have an internet connection that allows you to reach GitHubs servers and common package mirrors. 