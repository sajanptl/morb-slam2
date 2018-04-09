#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/se3_ops.h"
#include <iostream>
#include <deque>
#include <math.h>

// #include<Eigen/StdVector>
// May need to include unsupported for exp or else we can do it element by element
// #include<eigen3/unsupported/Eigen/MatrixFunctions>

#include "Converter.h"
#define PI 3.1415926536
#include<mutex>

// namespace ORB_SLAM2
// {
typedef Eigen::Matrix<double, 6, 1> Vector6d;
// typedef Matrix<double, 3, 1> Vector3d;

// Pose is the return type of the motion model
class MotionBase
{
public:
	MotionBase() {}
	~MotionBase() {}
	// virtual Eigen::Vector3d calc() = 0;
	// Eigen::Vector3d calc();
};

/* Result:
 *		Rwb_{t + dt}	Final IMU pose
 *		wv_{t + dt}		Final velocity in world
 *		wp_{t + dt}		Final pose in world
 *
 * Inputs:
 *		Rwb_{t}		Initial IMU pose (get this implicitly from acceleration and angular velocity)
 *		bowb_{t}	rotational velocity (last 3 in imu vector)
 *		wv_{t}		Initial velocity in world (integrated from imu)
 *		wa_{t}		Initial acceleration in world (first 3 in imu vector)
 *		wp_{t}		Initial pose in world (EdgeStereoSE3ProjectXYZOnlyPose.Xw which is type Vector3d)
 *		dt			IMU sampling frequency
 */
class KittiMotion : public MotionBase
{
public:
	KittiMotion()
	{
		p << 0,
			 0,
			 0;
		R << 1, 0, 0,
			 0, 1, 0,
			 0, 0, 1;
		v << 0,
			 0,
			 0;
		omega << 0,
				 0,
				 0;
		a << 0,
			 0,
			 0;
		ba << 0,
			  0,
			  0;
		bg << 0,
			  0,
			  0;
	}
	~KittiMotion() {}
	Eigen::Matrix3d exp(const Eigen::Vector3d &omega) {
		double theta = omega.norm();
		Eigen::Matrix3d Omega = g2o::skew(omega);
		Eigen::Matrix3d Omega2 = Omega * Omega;
		Eigen::Matrix3d R;
		if (theta < 0.00001) {
			R = (Eigen::Matrix3d::Identity() + Omega + Omega*Omega);
			return R;
		}
		R = (Eigen::Matrix3d::Identity()
			 + sin(theta) / theta * Omega + (1 - cos(theta)) / (theta * theta) * Omega2);
		return R;
	}
	g2o::SE3Quat calc(const Eigen::Vector3d &p0, const Eigen::Matrix3d &R0, const Eigen::Vector3d &v0, const Vector6d &imu, const double &dt) {
		omega[0] = imu[3];
		omega[1] = imu[4];
		omega[2] = imu[5];
		a[0] = imu[0];
		a[1] = imu[1];
		a[2] = imu[2];
		// Update equations
		cout << "R before:\n" << R << "\n";
		R = R0 * exp((omega - bg) * dt);
		cout << "R after:\n" << R << "\n";
		v = v0 + ((a - ba) * dt);
		p = p0 + (v0 * dt) + (0.5 * (a - ba) * pow(dt,2));
		g2o::SE3Quat result(R,p);
		return result;
	}
	g2o::SE3Quat calc(const Eigen::Vector3d &p0, const Eigen::Matrix3d &R0, const Eigen::Vector3d &v0, deque<Vector6d> &imus, const double &dt)
	{
		g2o::SE3Quat result;
		Eigen::Vector3d p_previous = p0;
		Eigen::Matrix3d R_previous = R0;
		for (int i = 0; i < int(imus.size() - 1); ++i)
		{
			result = calc(p_previous, R_previous, v0, imus[i], dt);
			p_previous = result.translation();
			R_previous = result.rotation().toRotationMatrix();
		}
		return result;
	}
	void setLinearBias(Eigen::Vector3d ba_in) {
		ba = ba_in;
		return;
	}
	void setAngularBias(Eigen::Vector3d bg_in) {
		bg = bg_in;
		return;
	}
private:
	Eigen::Vector3d p;
	Eigen::Matrix3d R;
	Eigen::Vector3d v;
	Eigen::Vector3d omega;
	Eigen::Vector3d a;
	Eigen::Vector3d ba;
	Eigen::Vector3d bg;
};

void stayTest()
{
	KittiMotion testObject;
	g2o::SE3Quat quat_result;
	Eigen::Vector3d p;
	Eigen::Vector3d p0;
	Eigen::Matrix3d R0;
	Eigen::Vector3d v0;
	Vector6d imu;
	p0 << 0,
		  0,
		  0;
	R0 << 1, 0, 0,
		  0, 1, 0,
		  0, 0, 1;
	v0 << 1,
		  0,
		  0;
	imu << 0, 0, 0, 0, 0, 0;
	double dt = 1;
	quat_result = testObject.calc(p0, R0, v0, imu, dt);
	cout << "Start:\n" << p0 << "\nEnd:\n" << quat_result.rotation().toRotationMatrix() << "\n";
	return;
}

int main(int argc, char const *argv[])
{
	stayTest();
	return 0;
}

// }