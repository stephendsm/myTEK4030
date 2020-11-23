#include <crustcrawler_lib/kinematics.h>

#include <cmath>

#include <stdio.h>
#include <iostream>

#include <eigen3/Eigen/Dense>

using namespace crustcrawler_lib;
using namespace Eigen;

Kinematics::Kinematics()
{
	m_n = 6;

	m_a.resize(m_n);
	m_d.resize(m_n);
	m_alpha.resize(m_n);
	m_theta_offset.resize(m_n);

	double L1 = 0.108;  // [m]
	double L2 = 0.220;  // [m]
	double L3 = 0.156;  // [m]
	double L4 = 0.080;  // [m]
	double L6 = 0.074;  // [m]

	m_a            <<        0.0,         L2,  L3,  L4,        0.0, 0.0;
	m_d            <<         L1,        0.0, 0.0, 0.0,        0.0,  L6;
	m_alpha        << -M_PI/2.0f,        0.0, 0.0, 0.0,  M_PI/2.0f, 0.0;
	m_theta_offset <<        0.0, -M_PI/2.0f, 0.0, 0.0,  M_PI/2.0f, 0.0;
}

Kinematics::Kinematics(VectorXd a, VectorXd d, VectorXd alpha, VectorXd theta_offset) :
 m_n(a.size()),
 m_a(a),
 m_d(d),
 m_alpha(alpha),
 m_theta_offset(theta_offset)
{

}

Kinematics::~Kinematics()
{

}

Matrix4d Kinematics::getForwardTransform(int i, double theta_i)
{
	Matrix4d T;

	if ( i-1 > m_n )
		return Matrix4d::Identity();

	double ct = cos(theta_i+m_theta_offset(i-1));
	double st = sin(theta_i+m_theta_offset(i-1));
	double ca = cos(m_alpha(i-1));
	double sa = sin(m_alpha(i-1));
	double a = m_a(i-1);
	double d = m_d(i-1);

	T << ct, -st*ca,  st*sa,  a*ct,
         st,  ct*ca, -ct*sa,  a*st,
         0,    sa,       ca,     d,
         0,    0,         0,       1;

	return T;
}

Matrix4d Kinematics::getForwardKinematics(Eigen::VectorXd theta)
{
	Matrix4d T;

	VectorXd q(m_n);
	q = theta;

	T = getForwardTransform(1,q(0));

	for(int i = 1; i<m_n; i++)
	{
		T = T*getForwardTransform(i+1,q(i));
	}

	return T;
}

Eigen::MatrixXd Kinematics::getJacobian(Eigen::VectorXd theta)
{
	MatrixXd J;
	Matrix4d T_i_0, T_N_0;

	VectorXd q(m_n);
	q = theta + m_theta_offset;

	T_i_0;
	T_i_0 << Matrix4d::Identity();
	T_N_0 = getForwardKinematics(q);

	J.resize(6,m_n);

	Vector3d z(0.0, 0.0, 1.0);
	Vector3d z_i_0(0.0, 0.0, 1.0);

	Vector3d o_i_0(0.0, 0.0, 0.0);
	Vector3d o_N_0;
	o_N_0 = T_N_0.block<3,1>(0,3);

	Matrix3d R_i_0, R_N_0;
	R_i_0 << Matrix3d::Identity();
	R_N_0 = T_N_0.block<3,3>(0,0);

	for(int i = 0; i<m_n; i++)
	{
		J.block<3,1>(0,i) = skew(z_i_0)*(o_N_0-o_i_0);
		J.block<3,1>(3,i) = z_i_0;

		T_i_0 = T_i_0*getForwardTransform(i+1,q(i));
		o_i_0 = T_i_0.block<3,1>(0,3);
		z_i_0 = T_i_0.block<3,3>(0,0)*z;

		//std::cout << "i:" << i+1 << std::endl << "o_i_0" << std::endl << o_i_0 << std::endl << "skew(z_i_0)" << std::endl << skew(z_i_0) << std::endl;
		//std::cout << "T_i_0" << std::endl << T_i_0 << std::endl;
	}

	return J;
}

void Kinematics::printDenavitHartenbergParameters()
{
	printf("%s\t\t%s\t\t%s\t\t%s\n","a","alpha","d","theta*");
	printf("%s\t\t%s\t\t%s\t\t%s\n","---","---","---","---");

	for(int i = 0; i<m_n; i++)
	{
		printf("%f\t%f\t%f\t%f\n",m_a(i),m_alpha(i),m_d(i),m_theta_offset(i));
	}
}