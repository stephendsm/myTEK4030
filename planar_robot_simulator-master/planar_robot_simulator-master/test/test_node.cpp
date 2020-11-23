#include <ros/ros.h>
#include <std_msgs/String.h>

#include <planar_robot_simulator/planar_robot_1dof.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(200);
  
  PlanarRobotSimulator::PlanarRobot1DOF sim;

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
	
	sim.step(1.0/200.0, 0.1);
	PlanarRobotSimulator::PlanarRobot1DOF::state_type state = sim.getState();
	
	ROS_INFO("%f %f", state[0], state[1]);
	
	sim.draw();

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}