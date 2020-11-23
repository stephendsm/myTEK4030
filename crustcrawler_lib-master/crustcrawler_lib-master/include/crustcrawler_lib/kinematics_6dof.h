#pragma once

#include <eigen3/Eigen/Core>

#include <crustcrawler_lib/kinematics.h>

namespace crustcrawler_lib
{
	/*!
	 * For all methods the frame convention is getTransformT_X_Y means \f$T_X^Y\f$
	 */
	class Kinematics6DOF : public Kinematics
	{
	public:
		Kinematics6DOF();
		~Kinematics6DOF();
		
		/*! Get the forward transform \f$T_{i}^{i-1}\f$ */
		//virtual Eigen::Matrix4d getForwardTransform(int i, double theta_i) override;
		
		/*! Get the forward kinematics \f$T_{N}^0\f$, where N is the number of DOF */
		virtual Eigen::Matrix4d getForwardKinematics(Eigen::VectorXd theta) override;
		
	private:
		double L1, L2, L3, L4, L6;
	};
	
}