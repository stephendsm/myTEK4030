#include <ros/ros.h>

#include <sensor_msgs/JointState.h> // since we are going to receive a 'JointState' msg

#include <planar_robot_simulator/planar_robot_2dof.h>

#include <eigen3/Eigen/Eigen>

#include <std_msgs/Float64MultiArray.h>

using namespace PlanarRobotSimulator;

using namespace PlanarRobotSimulator::Parameters;

std_msgs::Float64MultiArray control;

ros::Publisher command_pub;

Eigen::Vector2d e_i_m(0.0, 0.0);

Eigen::Vector2d e_m(0.0, 0.0);

double dt = 1.0/100.0;



//whenever the topic 'joint_state' is publish a message, we are going to 'jointStateCallback' function)
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO_THROTTLE(1, "Receiving messages");//receiving msg once per sec
    Eigen::Vector2d q_m(msg->position[0], msg->position[1]);
    Eigen::Vector2d qd_m(msg->velocity[0], msg->velocity[1]);

    //Eigen::Vector2d q_m_set(0.5*M_PI, -0.25*M_PI);//desired setting pointer
    Eigen::Vector2d temp(0.5*M_PI*PlanarRobotSimulator::Parameters::k_r_1, -0.25*M_PI*PlanarRobotSimulator::Parameters::k_r_2);
    Eigen::Vector2d q_m_set(PlanarRobot2DOF::q(temp));
    //Eigen::Vector2d K_p(1.0, 1.0);
    //Eigen::Vector2d u = K_p.cwiseProduct(q_m_set-q_m);
    e_m = q_m_set - q_m; //error
    e_i_m += dt*e_m; //integral

    Eigen::Vector2d K_p(0.12, 0.12); //assumed and modify
    Eigen::Vector2d T_p(4.5, 4.5); //assumed and modify

    Eigen::Vector2d u = K_p.cwiseProduct(e_i_m) + K_p.cwiseProduct(T_p).cwiseProduct(e_m);

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
  command_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_command", 1000);

  ros::Subscriber sub = nh.subscribe("joint_state", 1000, jointStateCallback);
  //spin call which causes the nodes to stop here and wait for incomming topics or services
  ros::spin();

  return 0;
}
