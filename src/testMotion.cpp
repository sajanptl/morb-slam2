#include <iostream>
#include "MotionModel.h"


void stayTest()
{
	cout << "One Step Stay Test\n";
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
	v0 << 0,
		  0,
		  0;
	imu << 1, 0, 0, 0, 0, 0;
	double dt = 1;
	quat_result = testObject.calc(p0, R0, v0, imu, dt);
	cout << "Start:\n" << p0 << "\n" << R0 << "\n";
	cout << "End:\n" << quat_result.translation() << "\n" << quat_result.rotation().toRotationMatrix() << "\n";
	return;
}

void stayTestIterative()
{
	cout << "Iterative Stay Test\n";
	KittiMotion testObject;
	g2o::SE3Quat quat_result;
	Eigen::Vector3d p;
	Eigen::Vector3d p0;
	Eigen::Matrix3d R0;
	Eigen::Vector3d v0;
	Vector6d imu;
	deque<Vector6d> imus;
	p0 << 0,
		  0,
		  0;
	R0 << 1, 0, 0,
		  0, 1, 0,
		  0, 0, 1;
	v0 << 1,
		  0,
		  0;
	imu << 1, 0, 0, 0, 0, 0;
	for (int i = 0; i < 10; ++i)
	{
		imus.push_back(imu);
	}
	double dt = 1;
	quat_result = testObject.calc(p0, R0, v0, imus, dt);
	cout << "Start:\n" << p0 << "\n" << R0 << "\n";
	cout << "End:\n" << quat_result.translation() << "\n" << quat_result.rotation().toRotationMatrix() << "\n";
	return;
}

int main(int argc, char const *argv[])
{
	// stayTest();
	stayTestIterative();
	return 0;
}

