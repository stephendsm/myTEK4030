# TEK4030 ROS tutorial - part 1

This part of the tutorial will cover setting up a ROS workspace and creating a hallo world node.

## Setting up the workspace

Before stating to write our nodes we will set up a workspace. A workspace is a collection of packages that will be compiled an set up together.

First we make a directory for out workspace in the terminal.

```
mkdir ros_intro_ws
cd ros_intro_ws
```

Then we crate the source directory (`src`), where all our packages with source code will be put.

```
mkdir src
cd src
```

Then we initialize the workspace using the command

```
catkin_init_workspace
```

Note that `catkin` is the build and package handling system in ROS.

### Creating a package

Next we want to create a package that will contain the source code for our nodes. In the `src` directory write

```
catkin_create_pkg tek4030_ros_intro
```

This will create the necessary files for `catkin` to find and compile our package. The file `package.xml` contain meta information about the package in XML format, and `CMakeLists.txt` contains information on how to compile the source code. `catkin` use CMake to manage the build process, and therefore the `CMakeLists.txt` uses CMake syntax with some custom function for ROS.

We are now ready to start writing out first node.

## Writing a Hallo world node

Before writing a publisher and subscriber, we will write a very basic "Hallo world" node.

In the `tek4030_ros_intro` directory create a source directory and a source file

```
mkdir src
touch src/hallo_world.cpp
```

Now edit `src/hallo_world.cpp` and add the following content.

```cpp
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hallo_world");
  ros::NodeHandle nh;

  ROS_INFO("Hallo world!");

  return 0;
}
```

We will have a closer look at the code later. For now we only need to know that the code line `ROS_INFO("Hallo world!");` will print "Hallo world!" to the terminal. The rest is mostly setting up a main program and ROS.

In order to compile and run the code we need to edit the file ```CMakeLists.txt``` in out package. We will get more into details about this file later. Right now we will just edit it in order to compile our program. Do the following edits in the file.

```diff
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
-find_package(catkin REQUIRED)
+find_package(catkin REQUIRED roscpp)
...
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
-# ${catkin_INCLUDE_DIRS}
+${catkin_INCLUDE_DIRS}
)
...
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
-# add_executable(${PROJECT_NAME}_node src/tek4030_ros_intro_node.cpp)
+add_executable(hallo_world src/hallo_world.cpp)
...
## Specify libraries to link a library or executable target against
-# target_link_libraries(${PROJECT_NAME}_node
-#   ${catkin_LIBRARIES}
-# )
+target_link_libraries(hallo_world
+  ${catkin_LIBRARIES}
+)
```

If you now type ```catkin_make``` in the terminal while standing in the workspace directory (```ros_intro_ws```), our program should compile.

```
~/ros_intro_ws$ catkin_make
```

After this the compilation is done, we can run our program. First we need to make all the ROS functionality, by setting the correct environment variables in the terminal. This is done with the following command, and must be done every time you start a new terminal. It is possible to automate this process.

```
~/ros_intro_ws$ source devel/setup.bash
```

After that we need to launch the ROS master, that manages all the ROS nodes. This is done by with the following command. Do this in a separate terminal.

```
roscore
```
Finaly we can launch our program using the following command

```
~/ros_intro_ws$ rosrun tek4030_ros_intro hallo_world
```

The output should be something like this

```
[ INFO] [1534251007.096893272]: Hallo world!
```
