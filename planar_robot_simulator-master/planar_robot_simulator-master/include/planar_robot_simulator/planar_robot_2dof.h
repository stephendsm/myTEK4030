#pragma once

#include <boost/numeric/odeint.hpp>

#include <eigen3/Eigen/Eigen>

namespace PlanarRobotSimulator
{
	
	namespace Parameters
	{
		// Example 7.2
		double a_1 = 1;
		double a_2 = 1;
		double l_1 = 0.5;
		double l_2 = 0.5;
		double m_l_1 = 50;
		double m_l_2 = 50;
		double I_l_1 = 10;
		double I_l_2 = 10;
		double k_r_1 = 100;
		double k_r_2 = 100;
		double m_m_1 = 5;
		double m_m_2 = 5;
		double I_m_1 = 0.01;
		double I_m_2 = 0.01;
		double f_v = 0.1;
		double gravity = 9.81;
		double f = 2;
		
		// Problem 5.2
		double k_t_1 = 0.2;
		double k_t_2 = 0.2;
		double r_a_1 = 0.2;
		double r_a_2 = 0.2;
		double k_v_1 = 0.2;
		double k_v_2 = 0.2;
		double g_v_1 = 50.0; // invented
		double g_v_2 = 50.0; // invented
	}
	
class PlanarRobot2DOF
{
public:
	typedef std::vector< double > state_type;
	
private:
	boost::numeric::odeint::runge_kutta4< state_type > rk;
	
	state_type state;
	Eigen::Vector2d u;
public:
	
	
	PlanarRobot2DOF();
	~PlanarRobot2DOF();
	
	void operator()( const state_type &x , state_type &dxdt , double dt );
	
	void system( const state_type &x , state_type &dxdt , double t ) const;
	void step(double dt, Eigen::Vector2d u);
	
	static Eigen::Matrix2d B(Eigen::Vector2d q);
	static Eigen::Matrix2d C(Eigen::Vector2d q, Eigen::Vector2d qd);
	static Eigen::Vector2d g(Eigen::Vector2d q);
	
	static Eigen::Matrix4d T_1_0(Eigen::Vector2d q); //! \f$T_1^0$\f
	static Eigen::Matrix4d T_2_0(Eigen::Vector2d q); //! \f$T_2^0$\f
	
	static Eigen::Vector2d q(Eigen::Vector2d q_m);
	static Eigen::Vector2d qd(Eigen::Vector2d qd_m);
	
	static Eigen::Matrix2d diag2(double d1, double d2);
	
	state_type getState();
	void setState(state_type state);
	
	Eigen::Vector2d getJointPosition();
	Eigen::Vector2d getJointVelocity();
	
	Eigen::Vector2d getMotorPosition();
	Eigen::Vector2d getMotorVelocity();
	
	void draw();
};

}