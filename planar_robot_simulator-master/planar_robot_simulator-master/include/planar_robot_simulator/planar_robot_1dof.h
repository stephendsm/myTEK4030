#pragma once

#include <boost/numeric/odeint.hpp>

#include <eigen3/Eigen/Eigen>

namespace PlanarRobotSimulator
{
	
class PlanarRobot1DOF
{
public:
	typedef std::vector< double > state_type;
	
private:
	boost::numeric::odeint::runge_kutta4< state_type > rk;
	
	state_type state;
	double u;
public:
	
	
	PlanarRobot1DOF();
	~PlanarRobot1DOF();
	
	void operator()( const state_type &x , state_type &dxdt , double dt );
	
	void system( const state_type &x , state_type &dxdt , double t ) const;
	void step(double dt, double u);
	
	state_type getState();
	void setState(state_type state);
	
	double getPosition() { return state[0]; }
	double getVelocity() { return state[1]; }
	
	void reset();
	
	void draw();
};

}