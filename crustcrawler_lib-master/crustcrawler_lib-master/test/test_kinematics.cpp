#include <gtest/gtest.h>

#include <crustcrawler_lib/kinematics.h>
#include <crustcrawler_lib/kinematics_6dof.h>

using namespace crustcrawler_lib;
using namespace Eigen;

TEST(Kinematics, printDenavitHartenbergParameters)
{
	Kinematics robot;
	
	robot.printDenavitHartenbergParameters();
}

TEST(Kinematics, getForwardKinematics)
{
	Kinematics robot;
	
	VectorXd theta(6);
	theta = VectorXd::Zero(6);
	theta.setRandom();
	
	std::cout << robot.getForwardKinematics(theta) << std::endl;
}

TEST(Kinematics, getJacobian)
{
	Kinematics robot;
	
	VectorXd theta(6);
	theta = VectorXd::Zero(6);
	theta.setRandom();
	
	std::cout << "theta" << std::endl << theta << std::endl << std::endl;
	
	std::cout << "Jacobian" << std::endl << robot.getJacobian(theta) << std::endl;
}

TEST(Kinematics6DOF, getForwardKinematics)
{
	Kinematics robot;
	Kinematics6DOF robot6;
	
	VectorXd theta(6);
	theta = VectorXd::Zero(6);
	theta.setRandom();
	
	Matrix4d T_1, T_2, T_sum;
	
	T_1 = robot.getForwardKinematics(theta);
	T_2 = robot6.getForwardKinematics(theta);
	T_sum = T_1-T_2;
	
	EXPECT_NEAR(T_sum.sum(), 0.0, 1E-6);
	
}