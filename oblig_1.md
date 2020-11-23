# Mandatory assignment 1 - Independent joint control in ROS

In this assignment we are expanding the simulator and controller from the tutorial. If you did not complete part 1 - 3, either do so or copy the solution in [part 3](intro_part_3.md). You could optionally also complete [part 4](intro_part_4.md) before doing the mandatory assignment.

## Updating the planar_robot_simulator repository

The `planar_robot_simulator` library might have changed since the tutorial, so you will need to check for updates before proceeding. Go to the ros_intro workspace and do the following commands

```
~/ros_intro_ws$ cd src
~/ros_intro_ws/src$ cd planar_robot_simulator/
~/ros_intro_ws/src/planar_robot_simulator$ git pull
```

Then write you user name and password to download the changes.

## Updating CMakeLists.txt

We are going to use a library for doing linear algebra in C++, and we need to add the header files to our project. The library is [Eigen](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html). Add the following in the `CMakeLists.txt` file:

```diff
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
+ ${EIGEN3_INCLUDE_DIR}
)
```

## Updating the simulation node

First of we need to change the simulation model in the node. Change the included header file to the 2 DoF version, like this

```diff
-#include <planar_robot_simulator/planar_robot_1dof.h>
+#include <planar_robot_simulator/planar_robot_2dof.h>
```

Then change the object to the new model, like this

```diff
-PlanarRobotSimulator::PlanarRobot1DOF sim;
+PlanarRobotSimulator::PlanarRobot2DOF sim;
```

In the new model we can get both motor position and joint position. The textbook have controllers that control on the motor variable, so we would like to publish the motor positions and velocities using the methods `getMotorPosition()` and `getMotorVelocity()`. These methods return an `Eigen::Vector2d` object. To access the content of the returned vector, see the example below.

```cpp
sensor_msgs::JointState msg;
msg.name.push_back("joint_1");
msg.name.push_back("joint_2");
msg.position.push_back(0.0);
msg.position.push_back(0.0);
msg.velocity.push_back(0.0);
msg.velocity.push_back(0.0);

Eigen::Vector2d q_m = sim.getMotorPosition();
msg.position[0] = q_m(0);
msg.position[1] = q_m(1);

Eigen::Vector2d q_m = sim.getMotorVelocity();
msg.velocity[0] = q_m(0);
msg.velocity[1] = q_m(1);
```

## Updating the control node

In the control node we need to update the control law t no handle two degrees of freedom. To do this we need to get data in `sensor_msgs::JointState` into two `Eigen::Vector2d` objects. This can be done as shown below

```cpp
Eigen::Vector2d q_m(msg->position[0], msg->position[1]);
Eigen::Vector2d qd_m(msg->velocity[0], msg->velocity[1]);
```

The gain matrix can also be expanded by using a `Eigen::Vector2d` object, and initialized as shown above. The control law can now be calculated using these objects.

```cpp
Eigen::Vector2d q_m_set(0.5*M_PI, -0.25*M_PI);
Eigen::Vector2d K_p(1.0, 1.0);
Eigen::Vector2d u = K_p.cwiseProduct(q_m_set-q_m);
```

Note that in the above control law we set the desired position for the motor positions, not the joints. In order to get the joint position we will use the `planar_robot_simulator` library. First include its header file, then we will use the namespace of that library, to make it out code easier to read. Write the following after all the include statements, but before the code starts.

```cpp
using namespace PlanarRobotSimulator;
```

We can now use static functions from the PlanarRobot2DOF object. Have a look at its header file (`planar_robot_simulator/planar_robot_2dof.h`) to find out which methods that we can use (all that are marked with static).

```cpp
Eigen::Vector2d q_m_set(PlanarRobot2DOF::q(0.5*M_PI), PlanarRobot2DOF::q(-0.25*M_PI));
Eigen::Vector2d K_p(1.0, 1.0);
Eigen::Vector2d u = K_p.cwiseProduct(q_m_set-q_m);
```

Since `q_m` and `q` are related with the gear reduction ratio we could instead calculate `q` using that factor instead. All parameters are available from the `planar_robot_simulator` library. Add the following namespace to the source file.

```cpp
using namespace PlanarRobotSimulator::Parameters;
```

We can now access all the parameters given in the header file `planar_robot_simulator/planar_robot_2dof.h`. We could now define the setpoint using the parameter instead.

```cpp
Eigen::Vector2d q_m_set(0.5*M_PI*k_r_1, -0.25*M_PI*k_r_2);
```

## Update on feedback message

We need to update the feedback message going from the controller to the simulator. Use the message `std_msgs::Float64MultiArray`, instead of the `std_msgs::Float64` message. Send two floats from the controller to the simulator. When the simulator receives the message, the two floats must be put into an `Eigen::Vector2d` and be given to the `step` method of the simulator, which no longer takes in a `double`, but an `Eigen::Vector2d`.

## Tips

### Implementing integration

Example on how to implement integration. The variable `e_i_m` is the integral of the error `e_m`. The integral is found by multiplying with the time period `dt`.

```cpp
ros::Rate loop_rate(100);
double dt = 1.0/100.0;

Eigen::Vector2d e_i_m(0.0, 0.0);

while (ros::ok())
{
  Eigen::Vector2d q_r(0.25*M_PI*k_r_1, -0.25*M_PI*k_r_2);
  Eigen::Vector2d q_m = sim.getMotorPosition();

  Eigen::Vector2d e_m = q_r - q_m;
  e_i_m += dt*e_m;
}
```
