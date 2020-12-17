#include <ros/ros.h>

#include <planar_robot_simulator/planar_robot_2dof.h>

#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Eigen>

#include <std_msgs/Float64MultiArray.h>

Eigen::Vector2d u;

//whenever the topic 'joint_command' is publish a message, we are going to 'jointcmdCallback' function)
void jointcmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    //ROS_INFO_THROTTLE(1, "Receiving messages");//receiving msg once per sec
    u(0) = msg->data[0];
    u(1) = msg->data[1];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulator");
  ros::NodeHandle nh;
  
  PlanarRobotSimulator::PlanarRobot2DOF sim;
  
  // Adding a publisher
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", 1000); //'joint_state' is topic
  
  ros::Subscriber sub = nh.subscribe("joint_command", 1000, jointcmdCallback); //Subscribe to get command from controller
  
  sensor_msgs::JointState msg;
  //msg.name.push_back("pendulum"); // All arrays in the msg are standard vectors in C++, 
                                    //so we can use 'push_back' methods which is availabe for vectors 
                                    //to add one entry into the vectors
  //msg.position.push_back(0.0);
  //msg.velocity.push_back(0.0);
  msg.name.push_back("joint_1");
  msg.name.push_back("joint_2");
  msg.position.push_back(0.0);
  msg.position.push_back(0.0);
  msg.velocity.push_back(0.0);
  msg.velocity.push_back(0.0);

  ros::Rate loop_rate(100); // it helps us create with a fix frequency (100 Hz in this case)
  
  while ( ros::ok() )
  {
      ros::spinOnce(); // allows ros to check for incomming topic or services
      
      // Adding one step in the simulator
      double dt = 1.0/100.0;
      //double u = 0.0;
      sim.step(dt,u);
      
      // extract the state from the simulator and put it into the 'msg'
      msg.header.stamp = ros::Time::now(); //getting time stamp when the 'msg' is call
      //msg.position[0] = sim.getPosition(); //accessing the 1st entry in the vector with simulator position
      //msg.velocity[0] = sim.getVelocity(); ////accessing the 1st entry in the vector with simulator velocity
      Eigen::Vector2d q_m = sim.getMotorPosition();
      msg.position[0] = q_m(0);
      msg.position[1] = q_m(1);

      Eigen::Vector2d qd_m = sim.getMotorVelocity();
      msg.velocity[0] = qd_m(0);
      msg.velocity[1] = qd_m(1);
      
      
      
      // Using the object we have created 'joint_state_pub' to publish the msg
      joint_state_pub.publish(msg);
      
      sim.draw(); // drawing the simulator 
      
      loop_rate.sleep(); // stop here and sleep so that the rate fix to 100 hz inside this while loop
  }
  
  return 0;
}
