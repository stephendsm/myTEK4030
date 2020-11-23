#include <gtest/gtest.h>

#include <crustcrawler_lib/dynamics_simple_6dof.h>

using namespace crustcrawler_lib;
using namespace Eigen;

TEST(Dynamics, getGravityVector)
{
	DynamicsSimple6DOF robot;
	
	VectorXd theta(6);
	theta = VectorXd::Zero(6);
	theta(1) = 1.57;

	VectorXd g = robot.getGravityVector(theta);
	
	std::cout << g << std::endl;
}