#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include <iostream>

#include<Eigen/StdVector>
// May need to include unsupported for exp or else we can do it element by element
#include<unsupported/Eigen/MatrixFunctions>

#include "Converter.h"

#include<mutex>

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 3, 1> Vector3d;

// Pose is the return type of the motion model
class MotionBase
{
public:
	MotionBase() {}
	~MotionBase() {}
	virtual Vector3d calc() = 0;
	Vector3d calc();
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
		p << 0, 0, 0;
	}
	~KittiMotion();
	Vector3d calc(const Vector3d &p0, const Eigen::Matrix3d &R0, const Vector3d &v0, const Vector6d &imu, const double &dt) {
		omega[0] = imu[3];
		omega[1] = imu[4];
		omega[2] = imu[5];
		a[0] = imu[0];
		a[1] = imu[1];
		a[2] = imu[2];
		// Update equations
		Vector3d exp_term = omega * dt;
		R = R0 * exp_term.exp();
		v = v0 + (a * dt);
		p = p0 + (v0 * dt) + (0.5 * a * pow(dt,2));
		return p;
	}
private:
	Vector3d p;
	Vector3d R;
	Vector3d v;
	Vector3d omega;
	Vector3d a;
};

namespace ORB_SLAM2
{
void testDoNothing()
{
	return;
}

int main(int argc, char const *argv[])
{
	KittiMotion testObject;
	cout << "Hello, World!\n";
	return 0;
}

}