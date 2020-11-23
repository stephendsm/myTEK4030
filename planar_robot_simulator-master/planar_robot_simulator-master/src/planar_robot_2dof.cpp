#include <planar_robot_simulator/planar_robot_2dof.h>

#include <functional>

#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <eigen3/Eigen/Dense>

using namespace PlanarRobotSimulator;
using namespace PlanarRobotSimulator::Parameters;

PlanarRobot2DOF::PlanarRobot2DOF()
{
	state.resize(4);
	state[0] = -0.5*M_PI*k_r_1;
	state[1] = 0.0;
	state[2] = 0.0;
	state[3] = 0.0;
}

PlanarRobot2DOF::~PlanarRobot2DOF()
{
	
}

Eigen::Matrix2d PlanarRobot2DOF::B(Eigen::Vector2d q)
{
	Eigen::Matrix2d Bmat;
	
	double b_11 = I_l_1 + m_l_1*l_1*l_1 + k_r_1*k_r_1*I_m_1 + I_l_2 + m_l_2*(a_1*a_1 + l_2*l_2 + 2*a_1*l_2*cos(q(1))) + I_m_2 + m_m_2*a_1*a_1;
	double b_12 = I_l_2 + m_l_2*(l_2*l_2 + a_1*l_2*cos(q(1))) + k_r_2*I_m_2;
	double b_22 = I_l_2 + m_l_2*l_2*l_2 + k_r_2*k_r_2*I_m_2;
	
	Bmat(0,0) = b_11;
	Bmat(0,1) = b_12;
	Bmat(1,0) = b_12;
	Bmat(1,1) = b_22;
	
	return Bmat;
}

Eigen::Matrix2d PlanarRobot2DOF::C(Eigen::Vector2d q, Eigen::Vector2d qd)
{
	Eigen::Matrix2d Cmat;
	
	double h = -m_l_2*a_1*l_2*sin(q(1));
	
	Cmat(0,0) = h*qd(1);
	Cmat(0,1) = h*(qd(0)+qd(1));
	Cmat(1,0) = -h*qd(0);
	Cmat(1,1) = 0.0;
	
	return Cmat;
}

Eigen::Vector2d PlanarRobot2DOF::g(Eigen::Vector2d q)
{
	Eigen::Vector2d gvec;
	
	double g_1 = (m_l_1*l_1 + m_m_2*a_1 + m_l_2*a_1)*gravity*cos(q(0)) + m_l_2*l_2*gravity*cos(q(0)+q(1));
	double g_2 = m_l_2*l_2*gravity*cos(q(0)+q(1));
	
	gvec(0) = g_1;
	gvec(1) = g_2;
	
	return gvec;
}

Eigen::Matrix4d PlanarRobot2DOF::T_1_0(Eigen::Vector2d q)
{
	Eigen::Matrix4d T;
	T.setZero();
	
	T(0,0) = cos(q(0));
	T(0,1) = -sin(q(0));
	T(1,0) = sin(q(0));
	T(1,1) = cos(q(0));
	
	T(2,2) = 1.0;
	
	T(0,3) = a_1*cos(q(0));
	T(1,3) = a_1*sin(q(0));
	
	T(3,3) = 1.0;
	
	return T;
}

Eigen::Matrix4d PlanarRobot2DOF::T_2_0(Eigen::Vector2d q)
{
	Eigen::Matrix4d T;
	T.setZero();
	
	T(0,0) = cos(q(0)+q(1));
	T(0,1) = -sin(q(0)+q(1));
	T(1,0) = sin(q(0)+q(1));
	T(1,1) = cos(q(0)+q(1));
	
	T(2,2) = 1.0;
	
	T(0,3) = a_1*cos(q(0)) + a_2*cos(q(0)+q(1));
	T(1,3) = a_1*sin(q(0)) + a_2*sin(q(0)+q(1));
	
	T(3,3) = 1.0;
	
	return T;
}

Eigen::Vector2d PlanarRobot2DOF::q(Eigen::Vector2d q_m)
{
	Eigen::Vector2d qvec;
	
	Eigen::Vector2d K_r_inv(1.0/k_r_1, 1.0/k_r_2);
	qvec = K_r_inv.asDiagonal()*q_m;
	
	return qvec;
}

Eigen::Vector2d PlanarRobot2DOF::qd(Eigen::Vector2d qd_m)
{
	Eigen::Vector2d qdvec;
	
	Eigen::Vector2d K_r_inv(1.0/k_r_1, 1.0/k_r_2);
	qdvec = K_r_inv.asDiagonal()*qd_m;
	
	return qdvec;
}

Eigen::Matrix2d PlanarRobot2DOF::diag2(double d1, double d2)
{
	Eigen::Matrix2d D2;
	D2 << d1, 0.0,
	       0.0, d2;
	
	return D2;
}

// system equation
void PlanarRobot2DOF::operator()( const state_type &x_in , state_type &dxdt_out , double dt_in )
{
	// Model (8.18) and (8.11)
	// state vector is [q_m_1, q_m_2, qd_m_1, dq_m_2]^T
	// control input u is K_t*R_a^-1*G_v*v_c, (8.12) should still hold
	
	Eigen::Vector4d x(x_in.data());
	
	Eigen::Vector2d q_m = x.head<2>();
	Eigen::Vector2d qd_m = x.tail<2>();
	
	Eigen::Matrix2d K_r_inv = diag2(1.0/k_r_1, 1.0/k_r_2);
	Eigen::Matrix2d K_r = diag2(k_r_1, k_r_2);
	Eigen::Matrix2d F_v = diag2(f_v, f_v);
	
	Eigen::Matrix2d K_t = diag2(k_t_1, k_t_2);
	Eigen::Matrix2d R_a = diag2(r_a_1, r_a_2);
	Eigen::Matrix2d R_a_inv = diag2(1.0/r_a_1, 1.0/r_a_2);
	Eigen::Matrix2d K_v = diag2(k_v_1, k_v_2);
	Eigen::Matrix2d G_v = diag2(g_v_1, g_v_2);
	
	Eigen::Vector4d dxdt;
	
	Eigen::Matrix2d B_eq = K_r_inv*B(q(q_m))*K_r_inv;
	Eigen::Matrix2d C_eq = K_r_inv*C(q(q_m), qd(qd_m))*K_r_inv;
	Eigen::Matrix2d F_eq = K_r_inv*F_v*K_r_inv;
	
	dxdt.head<2>() = qd_m;
	dxdt.tail<2>() = B_eq.inverse()*(u - K_t*R_a_inv*K_v*K_r*qd(qd_m) - C_eq*q_m - F_eq*q_m - K_r_inv*g(q(q_m)));
	
 	dxdt_out[0] = dxdt(0);
	dxdt_out[1] = dxdt(1);
	dxdt_out[2] = dxdt(2);
	dxdt_out[3] = dxdt(3);
}

void PlanarRobot2DOF::step(double dt, Eigen::Vector2d u)
{
	state_type in, out;
	double t = 0.0;

	this->u = u;
	
	rk.do_step(*this ,  state , t , dt );
}

PlanarRobot2DOF::state_type PlanarRobot2DOF::getState()
{
	return state;
}

void PlanarRobot2DOF::setState(PlanarRobot2DOF::state_type state)
{
	this->state = state;
}

Eigen::Vector2d PlanarRobot2DOF::getJointPosition()
{
	Eigen::Vector2d q_m(state[0], state[1]);
	return q(q_m);
}

Eigen::Vector2d PlanarRobot2DOF::getJointVelocity()
{
	Eigen::Vector2d qd_m(state[3], state[4]);
	return qd(qd_m);
}

Eigen::Vector2d PlanarRobot2DOF::getMotorPosition()
{
	Eigen::Vector2d q_m(state[0], state[1]);
	return q_m;
}

Eigen::Vector2d PlanarRobot2DOF::getMotorVelocity()
{
	Eigen::Vector2d qd_m(state[3], state[4]);
	return qd_m;
}

void PlanarRobot2DOF::draw()
{
	int w = 500;
	double l = 100;
	cv::Mat robot_image = cv::Mat::zeros( w, w, CV_8UC3 );
	
	Eigen::Vector2d j1 = T_1_0(getJointPosition()).col(3).head(2);
	Eigen::Vector2d j2 = T_2_0(getJointPosition()).col(3).head(2);
	
	int thickness = 2;
	int lineType = cv::LINE_8;

	cv::Point p0(w/2,w/2);
	cv::Point p1(w/2+l*j1(0),w/2 - l*j1(1));
	cv::Point p2(w/2+l*j2(0),w/2 - l*j2(1));
	
	cv::line( robot_image,
	p0,
	p1,
	cv::Scalar( 255, 0, 0 ),
	thickness,
	lineType );
	
	cv::line( robot_image,
	p1,
	p2,
	cv::Scalar( 0, 255, 0 ),
	thickness,
	lineType );
	
	cv::imshow( "Robot simulator", robot_image );
	cv::waitKey( 2 );
}