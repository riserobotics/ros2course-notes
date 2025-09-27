+++
title = "Home"
date = 2025-09-24T19:26:24+02:00
weight = 5
chapter = false
pre = "<b>X. </b>"
draft = false
+++

## An Introduction to ROS2

**Welcome to the RISE "An Introduction to ROS2" crash course!**

The goal of this website is to provide notes and references for people interested in developing code within the ROS2 framework for use on the RISE exoskeleton. The world of ROS2 and robotic programming as a whole is quite vast, and this introduction is not attempting to introduce you to every nook and cranny of it. This is - hopefully not literally - a crash course, a set of notes and tasks that will help you get up to speed with writing ROS2 based code.

There are several parts of the ROS2 framework that we dont use on the exoskeleton for various reasons. As a result, those will not be covered here, or at most just quickly glossed over. Some parts of the framework are of particular importance, and will therefore be looked on with greater attention to detail. If you want to learn more about the ROS2 framework as a whole, there are lots of [resources]() you can choose from, that take a much more holistic approach to ROS2 and robotic programming.  

---


We don't assume any prerequisites in robotic programming, or any of the tools named in this tutorial - even though it certainly helps! This tutorial is structured so that you can skip chapters and parts that deal with information you already know. Every chapter is presented in four different ways:

1. This **website** combines notes, graphs, images and code snippets. Wherever possible we make references to the code used on the current exoskeleton.

2. The **presentation slides** avaliable for download on the top of each chapter page. These are the slides used during the introduction workshop at the beginning of each semester.

3. The **video recordings** of these presentations, as well as some other topics that are worth it to show in greater detail.

4. Several **code examples** and notebooks for trying out the content by yourself on your own machine. All of these can be found in [this](https://github.com/riserobotics/ros2course) repository.

{{% notice warning %}}
As you will learn, ROS has been under development for a while. Right now, there are many *Distributions* of ROS that each are named after some species of turtle ([**Humble** Hawksbill](https://en.wikipedia.org/wiki/Hawksbill_sea_turtle), [**Jazzy** Jalisco](https://en.wikipedia.org/wiki/Jalisco_mud_turtle) and so on). However a few years back, the ROS project released a breaking change that remodelled a lot of the underlying architecture, this *version* is now formally called ROS2 and no longer just ROS, and it's what we are using in this project. For the sake of simplicity we will refer to these ROS2 versions as ROS, but be warned that tutorials from the original ROS, are not fully translatable to how modern ROS(2) works. So, if you are diving deeper into this topic and are trying to find further readings, make sure that the material is about ROS2 (and ideally the two most recent LTS versions "Humble" and "Jazzy") and not the original ROS. 
{{% /notice %}}


You are welcome to only take a look at some of these, or watch all of them. Not for every task in the semester there is a need to know every bit of what's explained in this tutorial, but having a general grasp on how things work is quite useful. 