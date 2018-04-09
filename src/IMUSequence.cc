#include "IMUSequence.h"

#include <iostream>
#include <algorithm>
#include <cmath>

using std::abs;
using std::lower_bound;
using std::upper_bound;
using std::vector;
using std::pair;
using std::make_pair;
using std::deque;
using std::mutex;
using std::lock_guard;
using Eigen::VectorXd;

namespace ORB_SLAM2
{

void IMUSequence::add(const VectorXd& u, double t)
{
	lock_guard<mutex> lck(mtx); // lock, will release mtx when going out of scope
	imuSeq.push_back(make_pair(t, u));
	if ((maxSeqLen != -1) && (static_cast<int>(imuSeq.size()) > maxSeqLen)) 
		imuSeq.pop_front();
}

deque<pair<double, VectorXd>> IMUSequence::operator()(double tmin, double tmax)
{
	lock_guard<mutex> lck(mtx); // lock, will release mtx when going out of scope
	
	// get bounds for tmin and tmax
	auto low = lower_bound(imuSeq.begin(), imuSeq.end(), tmin,
						   [](const pair<double, VectorXd>& a, const double& val)
						     { return a.first < val; });
	auto up  = upper_bound(imuSeq.begin(), imuSeq.end(), tmax,
						   [](const double& val, const pair<double, VectorXd>& a)
						     { return val < a.first; });
	
	deque<pair<double, VectorXd>> useq(low, up); // initialize from iterators
	
	constexpr double eps = 1e-6; // tolerance for equality of doubles
	auto interp = [](double t, double t0, double dt, const VectorXd& u0, const VectorXd& du, double eps)
					{ return (dt > eps) ? ((t - t0) / dt * du) + u0 : u0; };

	std::cout << tmin << " " << low->first << std::endl;
	if (abs(low->first - tmin) > eps) // tmin does not match low.first
	{
		std::cout << "Low mismatch" << std::endl;
		// adjust forward iterators to set up interpolation
		auto lowPrev = low;
		auto lowNext = low;
		if ((low->first < tmin) && ((low + 1) != imuSeq.end())) ++lowNext;
		else if (low != imuSeq.begin()) lowPrev = imuSeq.begin() + (low - imuSeq.begin() - 1);
		
		// perform interpolation
		auto ui = interp(tmin, lowPrev->first, lowNext->first - lowPrev->first,
					     lowPrev->second, lowNext->second - lowPrev->second, eps);
		
		if (low->first < tmin) useq.front() = make_pair(tmin, ui); // overwrite first pos with tmin to make it the lowest
		else useq.push_front(make_pair(tmin, ui)); // add to tmin to front to make it the lowest
	}
	else std::cout << "Low Match" << std::endl;

	// up is one past hi, so get pointer to hi (could be tmax or around tmax)
	auto hi = imuSeq.begin() + (up - imuSeq.begin() - 1);
	std::cout << tmax << " " << hi->first << std::endl;
	if (abs(hi->first - tmax) > eps) // tmax does not match hi.first
	{
		std::cout << "Hi mismatch" << std::endl;
		// adjust forward iterators to set up interpolation
		auto hiPrev = hi;
		auto hiNext = hi;
		if ((hi->first < tmax) && ((hi + 1) != imuSeq.end())) ++hiNext;
		else if (up != imuSeq.end()) hiPrev = imuSeq.begin() + (hi - imuSeq.begin() - 1);

		// preform interpolation
		auto ui = interp(tmax, hiPrev->first, hiNext->first - hiPrev->first,
						 hiPrev->second, hiNext->second - hiPrev->second, eps);

		if (hi->first > tmax) useq.back() = make_pair(tmax, ui); // overwrite last pos with tmax to make it the highest
		else useq.push_back(make_pair(tmax, ui)); // add tmax to back to make it the highest
	}
	else std::cout << "Hi match" << std::endl;	

	return useq;
}

} // close ORB_SLAM2 namespace
