#include <crustcrawler_lib/kinematics_6dof.h>

#include <cmath>

#include <stdio.h>
#include <iostream>

#include <eigen3/Eigen/Dense>

using namespace crustcrawler_lib;
using namespace Eigen;

Kinematics6DOF::Kinematics6DOF() : Kinematics()
{
	m_n = 6;

	m_a.resize(m_n);
	m_d.resize(m_n);
	m_alpha.resize(m_n);
	m_theta_offset.resize(m_n);

	L1 = 0.108;  // [m]
	L2 = 0.220;  // [m]
	L3 = 0.156;  // [m]
	L4 = 0.080;  // [m]
	L6 = 0.074;  // [m]

	m_a            <<        0.0,         L2,  L3,  L4,        0.0, 0.0;
	m_d            <<         L1,        0.0, 0.0, 0.0,        0.0,  L6;
	m_alpha        << -M_PI/2.0f,        0.0, 0.0, 0.0,  M_PI/2.0f, 0.0;
	m_theta_offset <<        0.0, -M_PI/2.0f, 0.0, 0.0,  M_PI/2.0f, 0.0;
}

Kinematics6DOF::~Kinematics6DOF()
{

}

Matrix4d Kinematics6DOF::getForwardKinematics(Eigen::VectorXd theta)
{
	Matrix4d T;
	
	VectorXd q(m_n);
	q = theta + m_theta_offset;
	
	double c1 = cos(q(0));
	double s1 = sin(q(0));
	double c2 = cos(q(1));
	double s2 = sin(q(1));
	double c23 = cos(q(1)+q(2));
	double s23 = sin(q(1)+q(2));
	double c234 = cos(q(1)+q(2)+q(3));
	double s234 = sin(q(1)+q(2)+q(3));
	double c2345 = cos(q(1)+q(2)+q(3)+q(4));
	double s2345 = sin(q(1)+q(2)+q(3)+q(4));
	double c6 = cos(q(5));
	double s6 = sin(q(5));
	
	T << c1*c2345*c6-s1*s6, -c1*c2345*s6-s1*c6, c1*s2345, L6*c1*s2345 + L4*c1*c234 + L3*c1*c23 + L2*c1*c2,
	     s1*c2345*c6+c1*s6, -s1*c2345*s6+c1*c6, s1*s2345, L6*s1*s2345 + L4*s1*c234 + L3*s1*c23 + L2*s1*c2,
		 -s2345*c6        , s2345*s6          , c2345  ,  L6*c2345 - L4*s234 - L3*s23 - L2*s2 + L1,
		 0.0,0.0,0.0,1.0;
		 
	return T;
}