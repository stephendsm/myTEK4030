#pragma once

#include <eigen3/Eigen/Core>

namespace crustcrawler_lib
{
	class DynamicsSimple6DOF
	{
	public:
		DynamicsSimple6DOF();
		~DynamicsSimple6DOF();
		
		/*! Get the gravity vector \f$g(q)\f$ */
		virtual Eigen::VectorXd getGravityVector(Eigen::VectorXd theta);
		
	protected:
		int m_n; /*!< Number of DOF for the robot */
		Eigen::VectorXd m_m_l; /*!< Link mass \f$m_l\f$ */
		Eigen::VectorXd m_m_m; /*!< Motor mass \f$m_m\f$ */
		Eigen::VectorXd m_theta_offset; /*!< Denavit-Hartenberg parameter \f$\theta_i\f$ */
		
		double L1, L2, L3, L4, L6;
	};
}