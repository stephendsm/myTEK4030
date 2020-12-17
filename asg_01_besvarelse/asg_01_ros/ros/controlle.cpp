#include <ros/ros.h>

#include <sensor_msgs/JointState.h> // since we are going to receive a 'JointState' msg

#include <planar_robot_simulator/planar_robot_2dof.h>

#include <eigen3/Eigen/Eigen>

#include <std_msgs/Float64MultiArray.h>

using namespace PlanarRobotSimulator;

std_msgs::Float64MultiArray control;

//whenever the topic 'joint_state' is publish a message, we are going to 'jointStateCallback' function)
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO_THROTTLE(1, "Receiving messages");//receiving msg once per sec
    Eigen::Vector2d q_m(msg->position[0], msg->position[1]);
    Eigen::Vector2d qd_m(msg->velocity[0], msg->velocity[1]);

    //Eigen::Vector2d q_m_set(0.5*M_PI, -0.25*M_PI);//desired setting pointer
    Eigen::Vector2d temp(0.5*M_PI, -0.25*M_PI);
    Eigen::Vector2d q_m_set(PlanarRobot2DOF::q(temp));
    Eigen::Vector2d K_p(1.0, 1.0);
    Eigen::Vector2d u = K_p.cwiseProduct(q_m_set-q_m);

    control.data[0] = u(0);
    control.data[1] = u(1);

    command_pub.publish(control);
}

int main(int argc, char **argv)
{

  control.data.push_back(0.0);
  control.data.push_back(0.0);

  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  
  //Adding a subscriber
  //ros::Subscriber joint_state_sub = nh.subscribe("joint_state", 1000, jointStateCallback);
  command_pub = nh.advertise<std_msgs:Float64MultiArray>("Joint_command", 1000);

  ros::Subscriber sub = nh.subscribe("Joint_State", 1000, jointStateCallback);
  //spin call which causes the nodes to stop here and wait for incomming topics or services
  ros::spin();

  return 0;
}

