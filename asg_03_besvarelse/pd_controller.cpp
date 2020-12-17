#include "/home/student/assignment3/src/crustcrawler_lib/include/crustcrawler_lib/dynamics_simple_6dof.h"
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <stddef.h>
#include <stdlib.h>
#include <eigen3/Eigen/Eigen>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/JointState.h"
#include <ros/ros.h>

using namespace crustcrawler_lib;
using namespace Eigen;

// Publish nodes for joint 1-6 (position)
ros::Publisher* pub_j1 = NULL;
ros::Publisher* pub_j2 = NULL;
ros::Publisher* pub_j3 = NULL;
ros::Publisher* pub_j4 = NULL;
ros::Publisher* pub_j5 = NULL;
ros::Publisher* pub_j6 = NULL;


void getJointStates(const sensor_msgs::JointState& joint_state)
{ 
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6); //qd= Eigen::VectorXd::Zero(6);     //Joint velocities
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(6); //q_d= Eigen::VectorXd::Zero(6);    //Desired joint position

    try{
        
        for (int i=0; i<6; i++) q(i) = joint_state.position[i];     //storing joint positions
        for (int j=0; j<6; j++) qd(j) = joint_state.velocity[j];    //storing joint velocities

    }
    catch (...){
        ROS_ERROR("could not find q or qd");
    }

    
    Eigen::VectorXd u = Eigen::VectorXd::Zero(6);   //u from PD with gravity compensation
    Eigen::VectorXd K_p = Eigen::VectorXd::Zero(6); //Proportional gain
    VectorXd K_d = VectorXd::Zero(6);               //Derivative gain
    Eigen::VectorXd q_d(6);                         //Desired joint position
	
    //Set desired joint positions
    q_d(0) = 0;
    q_d(1) = 0;
    q_d(2) = 0;
    q_d(3) = 1;
    q_d(4) = 0;
    q_d(5) = 0;

    //Gains
    double kp = 1.0;
    double kd = 0.4;

    K_p.setConstant(kp);
    K_d.setConstant(kd);

    crustcrawler_lib::DynamicsSimple6DOF robot;
 	Eigen::VectorXd g = robot.getGravityVector(q);        //Gravity

    //PD Controller 
    u = g + K_p.asDiagonal()*(q_d-q) - K_d.asDiagonal()*qd;

    std_msgs::Float64 output1; std_msgs::Float64 output2; std_msgs::Float64 output3;
    std_msgs::Float64 output4; std_msgs::Float64 output5; std_msgs::Float64 output6;

    output1.data = float(u(0)); output2.data = float(u(1)); output3.data = float(u(2));
    output4.data = float(u(3)); output5.data = float(u(4)); output6.data = float(u(5));

    pub_j1->publish(output1); pub_j2->publish(output2); pub_j3->publish(output3);
    pub_j4->publish(output4); pub_j5->publish(output5); pub_j6->publish(output6);

}


int main(int argc, char **argv)
{
    //Start controller node
    ros::init(argc, argv, "controller");
    ROS_INFO("Starting controller");
    ros::NodeHandle nh;

    //Allocate subscribers
    ros::Subscriber sub_joint_states = nh.subscribe("/crustcrawler/joint_states", 100, getJointStates);


    //Allocate publishers - Publish joints to crustcrawler
    ros::Publisher j1 = nh.advertise<std_msgs::Float64>("crustcrawler/joint1_controller/command", 100);
    ros::Publisher j2 = nh.advertise<std_msgs::Float64>("crustcrawler/joint2_controller/command", 100);
    ros::Publisher j3 = nh.advertise<std_msgs::Float64>("crustcrawler/joint3_controller/command", 100);
    ros::Publisher j4 = nh.advertise<std_msgs::Float64>("crustcrawler/joint4_controller/command", 100);
    ros::Publisher j5 = nh.advertise<std_msgs::Float64>("crustcrawler/joint5_controller/command", 100);
    ros::Publisher j6 = nh.advertise<std_msgs::Float64>("crustcrawler/joint6_controller/command", 100);

    pub_j1 = &j1; pub_j2 = &j2; pub_j3 = &j3;
    pub_j4 = &j4; pub_j5 = &j5; pub_j6 = &j6;
    
    ros::spin();

    return 0;
}
