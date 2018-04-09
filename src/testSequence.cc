#include <iostream>
#include "IMUSequence.h"
#include <Eigen/Dense>
#include <deque>
#include <utility>

using namespace ORB_SLAM2;
using namespace std;
using namespace Eigen;

int main()
{
	cout << "init" << endl;
	IMUSequence imuSeq;
	
	VectorXd u0(2); u0 << 0, 0;
	VectorXd u1(2); u1 << 1, 1;
	VectorXd u2(2); u2 << 2, 2;
	VectorXd u3(2); u3 << 3, 3;
	double t0{0}, t1{1}, t2{2}, t3{3};
	
	cout << "adding" << endl;
	imuSeq.add(u0, t0);
	imuSeq.add(u1, t1);
	imuSeq.add(u2, t2);
	imuSeq.add(u3, t3);

	cout << "get 0-3" << endl;
	auto res = imuSeq(0, 3);
	for (const auto& r : res)
		cout << r.first << " " << r.second.transpose() << endl;
	
	cout << "get 0.5-2.5" << endl;
	res = imuSeq(0.5, 2.5);
	for (const auto& r : res)
		cout << r.first << " " << r.second.transpose() << endl;
	
	cout << "get -0.5-3.5" << endl;
	res = imuSeq(-0.5, 3.5);
	for (const auto& r : res)
		cout << r.first << " " << r.second.transpose() << endl;
	
	return 0;
}
