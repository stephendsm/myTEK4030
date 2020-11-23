#include <crustcrawler_lib/dynamics_simple_6dof.h>

#include <cmath>

#include <stdio.h>
#include <iostream>

#include <eigen3/Eigen/Dense>

using namespace crustcrawler_lib;
using namespace Eigen;

DynamicsSimple6DOF::DynamicsSimple6DOF()
{
	m_n = 6;

	m_m_l.resize(m_n);
	m_m_m.resize(m_n);

	L1 = 0.108;  // [m]
	L2 = 0.220;  // [m]
	L3 = 0.156;  // [m]
	L4 = 0.080;  // [m]
	L6 = 0.074;  // [m]

	m_m_l << 0.0, 0.0, 1.3, 0.8, 0.0, 0.0; // [ox]
	m_m_m << 6.0, 13.2, 6.0, 6.0, 3.4, 3.4; // [oz]
	
	m_m_l *= 9.81*0.0283495231; // N
	m_m_m *= 9.81*0.0283495231; // N
	
	m_theta_offset.resize(m_n);
	m_theta_offset << 0.0, -M_PI/2.0f, 0.0, 0.0,  M_PI/2.0f, 0.0;
}

DynamicsSimple6DOF::~DynamicsSimple6DOF()
{

}

Eigen::VectorXd DynamicsSimple6DOF::getGravityVector(Eigen::VectorXd theta)
{
	VectorXd g(m_n);
	
	VectorXd q(m_n);
	q = theta + m_theta_offset;
	
	double c2 = cos(q(1));
	double c23 = cos(q(1)+q(2));
	double c234 = cos(q(1)+q(2)+q(3));
	double s2345 = sin(q(1)+q(2)+q(3)+q(4));
	
	VectorXd g_l(m_n);
	VectorXd g_m(m_n);
	
	g_m << 0.0,
	       L2*c2*m_m_m(2) + (L3*c23+L2*c2)*m_m_m(3) + (L4*c234+L3*c23+L2*c2)*m_m_m(4) + (L6*s2345+L4*c234+L3*c23+L2*c2)*m_m_m(5),
	       (L3*c23)*m_m_m(3) + (L4*c234+L3*c23)*m_m_m(4) + (L6*s2345+L4*c234+L3*c23)*m_m_m(5),
	       (L4*c234)*m_m_m(4) + (L6*s2345+L4*c234)*m_m_m(5),
		   (L6*s2345)*m_m_m(5),
		   0.0;
	
	g_l << 0.0,
	       0.5*L2*c2*m_m_l(2) + (0.5*L3*c23+L2*c2)*m_m_l(3) + (0.5*L4*c234+L3*c23+L2*c2)*m_m_l(4) + (0.5*L6*s2345+L4*c234+L3*c23+L2*c2)*m_m_l(5),
	       (0.5*L3*c23)*m_m_l(3) + (0.5*L4*c234+L3*c23)*m_m_l(4) + (0.5*L6*s2345+L4*c234+L3*c23)*m_m_l(5),
	       (0.5*L4*c234)*m_m_l(4) + (0.5*L6*s2345+L4*c234)*m_m_l(5),
		   (0.5*L6*s2345)*m_m_l(5),
		   0.0;
		   
	g = g_l + g_m;
	
	return g;
 }