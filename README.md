# TEK4030 ROS Intro

In this course we will be using **ROS**, an open-source, meta-operating system for robots.

<div align="center">
   <br>
  <img src="img\ROS-Development-BlogPost-01-ARTC+Update.png"><br><br>
</div>

ROS provides all the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers.

This tutorial accompanies our ROS introduction lecture. It will teach you to write a basic publisher and subscriber node, and how to dynamically change parameters inside one of the nodes. One can use both Python and C++ to write ROS application, but this course will focus on writing C++ code.


## Environments


<div align="center">
   <br>
  <img src="img\vmware-horizon-cloud-overview-video.jpg"><br><br>
</div>

Before we start the tutorial, we need to fix our setup.
To save time on installation and setup config, we have allocated dedicated Ubuntu (18.04) machines with ROS installed. You can log in to these machines using [VMware Horizon client](https://my.vmware.com/web/vmware/info?slug=desktop_end_user_computing/vmware_horizon_clients/4_0) . Add the server: `vdi-apcon-prod.uio.no` and then login using your university credentials.

Additionally if you use a US keyboard layout you can use a [browser](https://vdi-apcon-prod.uio.no)-[based](http://ros.ifi.uio.no) web client (there are bugs with other keyboard layouts).

All Linux and Mac machines at IFI has a VMware Horizon client installed with the server already added, so you can work from there if needed.

To access your home directory you might need to run the following command

```
systemctl start --user uio-vdi-automount
```

### Installing ROS on your own computer

You can also install ROS on your own machine by following [this](http://wiki.ros.org/melodic/Installation) guide.

### Using a virtual machine

It is also possible to use a virtual machine to run Ubuntu with ROS. Then you can run ROS from any OS. We have prepared a virtual machine that you may download and use by completing the following steps:

1. Install [Virtual Box](www.virtualbox.org)
2. Download the virtual machine [http://folk.uio.no/kimmat/ubuntu/](http://folk.uio.no/kimmat/ubuntu/)
3. Run the virtual machine

## Tutorial

 - [Part 1: Setting up a ROS workspace and creating a hallo world node](intro_part_1.md)
 - [Part 2: Writing a publisher node and viewing the published message](intro_part_2.md)
 - [Part 3: Writing a subscriber node, viewing topic connections and launching multiple nodes](intro_part_3.md)
 - [Part 4: (Optional) Creating a service, launch parameters and dynamic parameters](intro_part_4.md)


## Additional content (optional)

- [ROS tutorials from ros.org](http://wiki.ros.org/ROS/Tutorials)
  - [Launch files](http://wiki.ros.org/roslaunch)
  - [ROS bag](http://wiki.ros.org/rosbag)
  - [rqt_reconfigure](http://wiki.ros.org/dynamic_reconfigure/Tutorials)
  - [rqt](http://wiki.ros.org/rqt)
- [A Gentle Introduction to ROS](https://www.cse.sc.edu/~jokane/agitr/) by Jason M. O'Kane
- [CMake tutorial](https://cmake.org/cmake-tutorial/)

If you want to go more in depth on **ROS**, you can check out [the original paper on ROS](http://www.robotics.stanford.edu/~ang/papers/icraoss09-ROS.pdf). This can also work as a nice recap/introduction, depending on your knowledge level of **ROS**.

