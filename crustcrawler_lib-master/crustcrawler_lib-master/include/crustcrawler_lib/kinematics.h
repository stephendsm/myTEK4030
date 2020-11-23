#pragma once

#include <eigen3/Eigen/Core>

namespace crustcrawler_lib
{
	/*!
	 * For all methods the frame convention is getTransformT_X_Y means \f$T_X^Y\f$
	 */
	class Kinematics
	{
	public:
		Kinematics();
		Kinematics(Eigen::VectorXd m_a, Eigen::VectorXd m_l, Eigen::VectorXd m_alpha, Eigen::VectorXd m_theta_offset);
		~Kinematics();
		
		/*! Get the forward transform \f$T_{i}^{i-1}\f$ */
		virtual Eigen::Matrix4d getForwardTransform(int i, double theta_i);
		
		/*! Get the forward kinematics \f$T_{N}^0\f$, where N is the number of DOF */
		virtual Eigen::Matrix4d getForwardKinematics(Eigen::VectorXd theta);
		
		/*! Get the Jacobian \f$J_{N}^0\f$, where N is the number of DOF */
		virtual Eigen::MatrixXd getJacobian(Eigen::VectorXd theta);
		
		
		void printDenavitHartenbergParameters();
		
		Eigen::Matrix3d skew(Eigen::Vector3d v)
		{
			Eigen::Matrix3d M;
			M <<   0.0, -v(2),  v(1),
			      v(2),   0.0, -v(0),
			     -v(1),  v(0),   0.0;
			return M;
		}
		
	protected:
		int m_n; /*!< Number of DOF for the robot */
		Eigen::VectorXd m_a; /*!< Denavit-Hartenberg parameter \f$a_i\f$ */
		Eigen::VectorXd m_d; /*!< Denavit-Hartenberg parameter \f$d_i\f$ */
		Eigen::VectorXd m_alpha; /*!< Denavit-Hartenberg parameter \f$\alpha_i\f$ */
		Eigen::VectorXd m_theta_offset; /*!< Denavit-Hartenberg parameter \f$\theta_i\f$ */
	};
}