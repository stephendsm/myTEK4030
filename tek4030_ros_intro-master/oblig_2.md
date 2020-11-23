# Mandatory assignment 2 - Visual servoing using ROS

In this assignment we will simulate a robot holding a camera, and use a control law to move the camera so that a set of features reaches their desired value.

## Setting up the simulation

The simulator is lockated on the (github)[https://github.uio.no/TEK4030] page. To download it use the following commands

```
~/ros_intro_ws$ cd src
~/ros_intro_ws/src$ git clone https://github.uio.no/TEK4030/tek4030_visual_servoing.git
```

Then write you user name and password to download the changes. Now compile the simulator:

```
~/ros_intro_ws$ catkin_make
```

You can start the simulator by the following command.

```
~/ros_intro_ws$ roslaunch tek4030_visual_servoing_camera_controller simulation.launch
```

When starting the simulator you will start three different nodes.
 1. Gazebo - The simulator which simulates a 3D world with a plate with four circular markes. It also simulates a camera.
 2. Point tracking - An image processing node extracting the mid points of the four circular markers.
 3. Camera controller - An interface note to Gazebo to set a twist to the camera.

## Updating the simulator

Bacause of a bug in OpenCV for ROS in Ubuntu 18.04 the image processing node did not work properly. If you have this problem please update the simulator using the following command

```
~/ros_intro_ws/src/tek4030_visual_servoing$ git push
```

