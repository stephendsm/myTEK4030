#include <planar_robot_simulator/planar_robot_1dof.h>

#include <functional>

#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace PlanarRobotSimulator;

PlanarRobot1DOF::PlanarRobot1DOF()
{
	state.resize(2);
	state[0] = 0.99*M_PI;
	state[1] = 0.0;
}

PlanarRobot1DOF::~PlanarRobot1DOF()
{
	
}

// system equation
void PlanarRobot1DOF::operator()( const state_type &x , state_type &dxdt , double dt )
{
	// Model
	// I xdd + b xd + mgl sin x = u
	double m=1.0;
	double l=0.5;
	double r=0.1;
	double b= 0.1;
	double J=m*l*l; // point mass
	double g = 9.81;
	dxdt[0] = x[1];
	dxdt[1] = (u - m*l*g*sin(x[0]) - b*x[1])/J;
}

void PlanarRobot1DOF::step(double dt, double u)
{
	state_type in, out;
	double t = 0.0;

	this->u = u;
	
	rk.do_step(*this ,  state , t , dt );
}

PlanarRobot1DOF::state_type PlanarRobot1DOF::getState()
{
	return state;
}

void PlanarRobot1DOF::setState(PlanarRobot1DOF::state_type state)
{
	this->state = state;
}

void PlanarRobot1DOF::reset()
{
	state[0] = 0.99*M_PI;
	state[1] = 0.0;
}

void PlanarRobot1DOF::draw()
{
	int w = 400;
	double l = 100;
	cv::Mat robot_image = cv::Mat::zeros( w, w, CV_8UC3 );
	
	int thickness = 2;
	int lineType = cv::LINE_8;

	cv::line( robot_image,
	cv::Point(w/2,w/2),
	cv::Point(w/2+l*sin(state[0]),w/2 + l*cos(state[0])),
	cv::Scalar( 255, 0, 0 ),
	thickness,
	lineType );
	
	cv::imshow( "Robot simulator", robot_image );
	cv::waitKey( 2 );
}
