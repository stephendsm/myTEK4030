# TEK4030 ROS tutorial - part 2

This part of the tutorial will cover writing a publisher and viewing its output.

Before starting this part please read the ROS tutorial given in sources.

> **Sources**
>
> [Writing Publisher Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

We will write slightly different publisher and subscriber nodes, and use them throughout this tutorial. One will be a simulator for a pendulum, and the other will control that pendulum. In this part we will give small exercises that you should complete, and not explain every detail. How to do this exercises is found in the ROS tutorial.

You could also consider setting up an IDE (Integrated Development Environment) for you ROS development. KDevelop is a good choice, but you are free to choose any IDE. A list of different IDEs and how to set them up for ROS is given here

 * [KDevelop setup](kdevelop.md)
 * [ROS supported IDEs and setup](http://wiki.ros.org/IDEs)

## Writing a publisher node (simulator)

First we will write a simulator node. This simulator will be based on a library, and we will just make a simulation object in our code. First we will download our library to the source directory of our workspace. This is done using the following command (note that everything before the '$' sign indicates the directory we are in when executing the command).

```
~/ros_intro_ws/src$ git clone https://github.uio.no/TEK4030/planar_robot_simulator.git
```

Now supply your UiO username and password in order to download the package. It is possible to set up SSH keys for easier access of `github.uio.on` content, but we will not cover this.

After downloading try to compile the package. Compiling your ROS workspace was covered in Part 1 of the tutorial. It is done with the following command (in case you do not remember).

```
~/ros_intro_ws$ catkin_make
```

### Adding the simulator object

We will start with a bare node, and then add components to the node. First create the file `simulator.cpp` in the `src` directory in the `tek4030_ros_intro` package. Then add the following content.

```cpp
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
```

> **Exercise**
>
> Add the `simulator.cpp` source file to `CMakeLists.txt`, so that a separate executable node is compiled. Try to compile your code.

<details><summary>Solution</summary>
<p>

Add the following lines to `CMakeLists.txt`
```cmake
add_executable(simulator src/simulator.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})
```

</p>
</details>

We will now add the library and simulator to our node. Modify the source code as follows.

```diff
#include <ros/ros.h>

+#include <planar_robot_simulator/planar_robot_1dof.h>
+
int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh;

+  /* This is the object that simulates a pendulum */
+  PlanarRobotSimulator::PlanarRobot1DOF sim;
+
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();

+    /* This performs one step in the simulation */
+    double dt = 1.0/100.0;
+    double u = 0.0;
+    sim.step(dt, u);

+    /* This creates a window that shows the pendulum */
+    sim.draw();

    loop_rate.sleep();
  }

  return 0;
}
```

Then we need to update `CMakeLists.txt` to include the library. Fist of all we need to add a dependency to the `planar_robot_simulator` library. Then we also need to find the packages `OpenCV` (an image processing library) and to `Eigen3` (a linear algebra library). This is done with the following modifications to the `CMakeLists.txt` file.

```diff
-find_package(catkin REQUIRED roscpp)
+find_package(catkin REQUIRED roscpp planar_robot_simulator)
+find_package(OpenCV 3 REQUIRED)
+find_package(Eigen3 REQUIRED)
...
-# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
+add_dependencies(simulator ${catkin_EXPORTED_TARGETS})
...
-target_link_libraries(simulator ${catkin_LIBRARIES})
+target_link_libraries(simulator ${catkin_LIBRARIES} ${OpenCV_LIBS})
```

> **Exercise**
>
> Compile and run the node.

<details><summary>Solution</summary>
<p>

```
~/ros_intro_ws$ catkin_make
~/ros_intro_ws$ rosrun tek4030_ros_intro simulator
```

</p>
</details>

### Publishing the pendulum state

Now we are going to publish the joint state of the pendulum. ROS has a large set of build in message type, and we are going to use one of those now. The message [sensor_msg/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) contains the state for a n degree of freedom robot. In our case it will contain the angular position and velocity of the pendulum. First we will create the message in our node, and then we will write the code to publish it.

First we need to include the definition of the `JointState` message. This is doen my adding an `#include` statement at the top of our source file.

```cpp
#include <sensor_msgs/JointState.h>
```

Then we need to allocate memory for our message. It is possible to do this in the `while`-loop of our node, but it is faster to preallocate memory for our message. This is done by adding the following code before the `while`-loop.

```cpp
/* Allocate the message */
sensor_msgs::JointState msg;
msg.name.push_back("pendulum");
msg.position.push_back(0.0);
msg.velocity.push_back(0.0);
```

The first line defines the `msg` variable to be a `JointState` message. The fields `name`, `position` and `velocity` are arrays. In C++ these are implemented as [`std::vector` objects](http://www.cplusplus.com/reference/vector/vector/), which is a container type. This object has a set of methods to add and remove content to the container. Above we use `push_back` to add things to the back of the vector.

Then we need to update the message for each simulation step. This is done with the following code.

```cpp
/* Set the joint state */
msg.header.stamp = ros::Time::now();
msg.position[0] = sim.getPosition();
msg.velocity[0] = sim.getVelocity();
```

ROS uses headers in many of the messages. This is to get a time stamp of when the message was created. The first line sets the time stamp to now. The two other lines access the first object in the position and velocity vectors, and assign them with a position and velocity from the simulator.

> **Exercise**
>
> Write the necessary code to publish the `JointState` message. Publish the on the topic name "joint_state". Compile and run the simulator.

<details><summary>Solution</summary>
<p>

```cpp
ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", 1000);
...
while (ros::ok())
{
  ...
  joint_state_pub.publish(msg);
  ...
}
```

</p>
</details>

## Viewing the published message

We are now finished writing the simulator. Before we move on to the controller node we would like to verify that our message is published. To do this we are going to use the command line tools in ROS.

First we will check that our node is actually running. Type the following in a new terminal after starting the node.

```
rosnode list
```

This should produce the output

```
/rosout
/simulator
```

To get more details about the node we can type

```
rosnode info /simulator
```
Which produces this result.
```
--------------------------------------------------------------------------------
Node [/simulator]
Publications:
 * /joint_state [sensor_msgs/JointState]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services:
 * /simulator/get_loggers
 * /simulator/set_logger_level


contacting node http://MP039:33520/ ...
Pid: 14280
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
```

We see that the node publishes two topics, `/joint_state` and `/rosout`. The first is the topic we created, and the second is a standard topic all nodes publish.

To see all available topic we can write.

```
rostopic list
```

Which gives the following output.

```
/joint_state
/rosout
/rosout_agg
```

And again we see our topic available.

### Viewing message with rostopic

To check that the message is published are regular intervals we can write

```
rostopic hz /joint_state
```
This estimates the frequency which a message is published on a topic. The result should be something like this.

```
average rate: 99.989
	min: 0.010s max: 0.010s std dev: 0.00006s window: 100
```

To view the content of the message we can write.

```
rostopic echo /joint_state
```

This will continuously write out the content of the message. To avoid the running text we can add an option, like this.

```
rostopic echo -c /joint_state
```

This will return the following, where only the data changes values.

```
header:
  seq: 69369
  stamp:
    secs: 1534414825
    nsecs: 403059522
  frame_id: ''
name: [pendulum]
position: [-1.0937587154695192e-60]
velocity: [-3.8201627052885235e-60]
effort: []
---
```

### Viewing date vith rqt_plot

Writing `rqt_plot` in the terminal will open a plot tool window. Writing `/joint_state/position[0]` in the topic field and pressing enter (or clicking the + button) will add this topic to the plot. The topic field will suggest topics as you write. Press on the button with the magnifying glass to zoom.

> **Exercise**
>
> Restart the simulator and view the position response.
