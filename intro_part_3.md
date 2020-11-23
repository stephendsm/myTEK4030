# TEK4030 ROS tutorial - part 3

This part of the tutorial will cover writing a subscriber node, verifying that it is connected to the publisher and launching multiple nodes using `roslaunch`

> **Sources**
>
> [Writing Publisher Subscriber (C++)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

## Writing a subscriber node (controller)

We will start with a bare node, and then add components to the node. First create the file `controller.cpp` in the `src` directory in the `tek4030_ros_intro` package. Then add the following content.

```cpp
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  ros::spin();

  return 0;
}
```

> **Exercise**
>
> Add the `controller.cpp` source file to `CMakeLists.txt`, so that a separate executable node is compiled. Try to compile your code.

<details><summary>Solution</summary>
<p>

Add the following lines to `CMakeLists.txt`
```cmake
add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
```

</p>
</details>

### Adding a subscription

We will now add a subscriber to the node, by adding the following code.

```diff
#include <ros/ros.h>

+#include <sensor_msgs/JointState.h>

+void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
+{
+  ROS_INFO_THROTTLE(1, "Receiving messages");
+}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

+  ros::Subscriber sub = nh.subscribe("joint_state", 1000, jointStateCallback);

  ros::spin();

  return 0;
}
```

The `ROS_INFO_THROTTLE` statement does the same as `ROS_INFO`, except that the messages get throttled so that there is a 1 second delay from the last print out.

> **Exercise**
>
> Start roscore, the simulator node and the controller node in one terminal each.

<details><summary>Solution</summary>
<p>

```
~/ros_intro_ws$ source devel/setup.bash
~/ros_intro_ws$ roscore
```

```
~/ros_intro_ws$ source devel/setup.bash
~/ros_intro_ws$ rosrun tek4030_ros_intro simulator
```

```
~/ros_intro_ws$ source devel/setup.bash
~/ros_intro_ws$ rosrun tek4030_ros_intro controller
```

</p>
</details>

### Using rqt_graph the check connections between nodes

ROS has many GUI programs to aid in robot development, and most om the are prefixed `rqt_`. In part 2 we look at `rqt_plot`, and now we are going to look at `rqt_graph`. It displays nodes and how topics are published and subscribed. When you hover over a node or topic, all its connections are highlighted.

> **Exercise**
>
> Start roscore, the simulator node and the controller node in one terminal each (as in the previous exercise). Then start `rqt_graph` from a separate terminal.

<details><summary>Solution</summary>
<p>

![rqt_graph](img/rqt_graph.png "rqt_graph")

</p>
</details>

## Using ROS launch to deploy multiple nodes

Now that we have two nodes we see that it is a bit unpractical to start our entire setup. It involves several terminal, and takes a bit of time. ROS has a system for launching several nodes at the same time, an this is called [`roslaunch`](http://wiki.ros.org/roslaunch/XML).

By creating so called launch files, we can start several nodes at the same time. These launch files can be written with parameters, and one launch file can include other launch files. This creates a very flexible system for launching node. In this tutorial we are going to have a very simple example. Launch files are XML files that are parsed using the `roslaunch` command line tool.

First we create a `launch` directory within out package

```
~/ros_intro_ws/src/tek4030_ros_intro$ mkdir launch
~/ros_intro_ws/src/tek4030_ros_intro$ touch launch/simulation.launch
```

Then add the following content to the `launch/simulation.launch` file

```xml
<?xml version="1.0" ?>
<launch>
  <node name="simulator" pkg="tek4030_ros_intro" type="simulator" />
  <node name="controller" pkg="tek4030_ros_intro" type="controller" />
</launch>
```

Note that when using `roslaunch` a `roscore` instance will automatically be started (unless it is already running). However it might be beneficial to start `roscore` in a separate terminal, as all topic connections are reset when restarting `roscore`. This means that if you are plotting with `rqt_plot`, and `roscore` is restarted, you will have to set up the plot again.

> **Exercise**
>
> Start the simulator and controller nodes using the launch file. Use the command
> ```
> roslaunch tek4030_ros_intro simulation.launch
> ```

## Expanding the nodes with a feedback loop

Now we want to create a feedback system, so that the controller node sends a command to the simulator.

> **Exercise**
>
> Modify the simulator and controller nodes so that
>  * The topic `joint_command` is published by the controller node and subscribed by the simulator node. Use the message `std_msgs/Float64`.
>  * In the control node `main` function
>    * Initialize a publisher, which publish a `std_msgs/Float64` on the `joint_command` topic
>    * Make the publisher a global variable, so that we can publish from the `jointStateCallback` function. Example on how an object can be made global in C++ is
>      ```cpp
>      ros::Publisher* command_pub_global = NULL;
>      
>      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
>      {
>        command_pub_global->publish(...);
>      }
>      
>      int main(int argc, char **argv)
>      {
>        ...
>        ros::Publisher command_pub = nh.advertise<std_msgs::Float64>("joint_command", 1000);
>        command_pub_global = &command_pub;
>        ...
>      }
>      ```
>  * In the control node `jointStateCallback` function
>    * Extract the measured pendulum position as
>    * Create a desired angle variable (`double q_set = M_PI`) and a control gain variable (`double K_p = 1.0`)
>    * Calculate the control input (`double u`) as
>      ```
>      double u = K_p*(q_set-q);
>      ```
>  * In the simulator node
>    * Subscribe to the `joint_command` topic
>    * Write a callback function that stores the command in a global variable
>    * Set `u` given to the simulator to the content of the `std_msgs/Float64` message on the joint_command` topic
>
> After modifying the code, compile it and launch the simulation.

<details><summary>Solution</summary>
<p>

`CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(tek4030_ros_intro)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp planar_robot_simulator)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tek4030_ros_intro
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

add_executable(hallo_world src/hallo_world.cpp)
add_executable(simulator src/simulator.cpp)
add_executable(controller src/controller.cpp)

target_link_libraries(hallo_world
  ${catkin_LIBRARIES}
)

target_link_libraries(simulator
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

target_link_libraries(controller
  ${catkin_LIBRARIES}
)
```

`src/simulator.cpp`

```cpp
#include <ros/ros.h>

#include <planar_robot_simulator/planar_robot_1dof.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>

double u = 0.0;

void jointCommandCallback(const std_msgs::Float64::ConstPtr& msg)
{
  u = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh;

  /* This is the object that simulates a pendulum */
  PlanarRobotSimulator::PlanarRobot1DOF sim;

  ros::Rate loop_rate(100);

  /* Allocate publishers */
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", 1000);

  /* Allcate subscibers */
  ros::Subscriber sub = nh.subscribe("joint_command", 1000, jointCommandCallback);

  /* Allocate the message */
  sensor_msgs::JointState msg;
  msg.name.push_back("pendulum");
  msg.position.push_back(0.0);
  msg.velocity.push_back(0.0);

  while (ros::ok())
  {
    ros::spinOnce();

    /* This performs one step in the simulation */
    double dt = 1.0/100.0;
    sim.step(dt, u);

	/* Set the joint state */
	msg.header.stamp = ros::Time::now();
	msg.position[0] = sim.getPosition();
	msg.velocity[0] = sim.getVelocity();

	/* Publish the message*/
	joint_state_pub.publish(msg);

    /* This creates a window that shos the pendulum */
    sim.draw();

    loop_rate.sleep();
  }

  return 0;
}
```

`src/controller.cpp`

```cpp
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>

ros::Publisher* command_pub_global = NULL;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  double q = msg->position[0];
  double q_set = M_PI;
  double K_p = 1.0;
  double u = K_p*(q_set-q);

  std_msgs::Float64 output;
  output.data = u;

  command_pub_global->publish(output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  ros::Publisher command_pub = nh.advertise<std_msgs::Float64>("joint_command", 1000);
  command_pub_global = &command_pub;

  ros::Subscriber sub = nh.subscribe("joint_state", 1000, jointStateCallback);

  ros::spin();

  return 0;
}
```

</p>
</details>
