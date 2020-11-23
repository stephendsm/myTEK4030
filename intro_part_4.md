# TEK4030 ROS tutorial - part 4

This part of the tutorial will cover creating a service, launch parameters and dynamic parameters. This part will however not go through everything in detail, but give exercises and relevant sources on how to solve the exercises. It is optional to solve this part.

> **Sources**
>
> - Services
>   - [Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)
> - Parameters
>   - [Using Parameters in roscpp](http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters)
>   - [roslaunch/XML/param](http://wiki.ros.org/roslaunch/XML/param)
> - Dynamic reconfigure
>   - [How to Write Your First .cfg File](http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile)
>   - [Setting up Dynamic Reconfigure for a Node(cpp)](http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28cpp%29)

# Creating a reset services

In order to see the simulation again without stopping and starting all the nodes, we would like to create a restart service.

> **Exercise**
>
> Read the ROS tutorial *Writing a Simple Service and Client (C++)*. Then write a restart service (server) with the name "/simulation/restart" in the simulation node. In this service the reset method in the simulator should be invoked.
>
> Try to restart the simulation using the command `rosservice call /simulation/restart`

# Expand the launch file with parameters

When creating a node there are usually many parameters that one would like to change before deployment, in order to be able to use the same node for many applications.

> **Exercise**
>
> Read the tutorial on *Using Parameters in roscpp* and *roslaunch/XML/param*. Expand the launch file for the controller node and controller node to have the setpoint and gain as parameters.

# Dynamically changing control variables

> **Exercise**
>
> Read the tutorial on *How to Write Your First .cfg File* and *Setting up Dynamic Reconfigure for a Node(cpp)*. Expand the controller node to have the setpoint and gain as dynamic parameters.
