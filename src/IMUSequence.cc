#include "IMUSequence.h"

#include <algorithm>
#include <cmath>

using std::abs;
using std::lower_bound;
using std::upper_bound;
using std::vector;
using std::pair;
using std::deque;
using std::muxtex;
using std::lock_guard;
using Eigen::VectorXd;

void IMUSequence::add(const VectorXd& u, double t)
{
	lock_guard<mutex> lck(mtx);
	imuSeq.emplace_back({t, u});
	if ((maxSeqLen != -1) && (static_cast<int>(imuSeq.size()) > maxSeqLen)) 
		imuSeq.pop_front();
}

deque<pair<double, VectorXd>> operator()(double tmin, tmax)
{
	lock_guard<mutex> lck(mtx);
	
	// get bounds for tmin and tmax
	auto low = lower_bound(imuSeq.begin(), imuSeq.end(), tmin,
						   [](const pair<double, VectorXd>& a, const double& val)
						     { return a.first < val; });
	auto up  = upper_bound(imuSeq.begin(), imuSeq.end(), tmax,
						   [](const double& val, const pair<double, VectorXd>& a)
						     { return val < a.first; });
	
	deque<pair<double, VectorXd>> useq(low, up); // initialize from iterators
	
	constexpr double eps = 1e-6; // tolerance for equality of doubles
	auto interp = [](double t, double t0, double dt, double u0, double du, double eps)
					{ return (dt > eps) ? ((t - t0) / dt * du) + u0 : u0; };

	if (abs(low.first - tmin) > eps) // tmin does not match low.first
	{
		// adjust forward iterators to set up interpolation
		auto lowPrev = low;
		auto lowNext = low;
		if (low.first < tmin) ++lowNext;
		else if (low != imuSeq.begin()) lowPrev = imuSeq.begin() + (low - imuSeq.begin() - 1);
		
		// perform interpolation
		auto ui = interp(tmin, lowPrev.first, lowNext.first - lowPrev.first,
					     lowPrev.second, lowNext.second - lowPrev.second, eps);
		
		if (low.first < tmin) useq.front() = {tmin, ui}; // overwrite first pos with tmin to make it the lowest
		else useq.emplace_front({tmin, ui}); // add to tmin to front to make it the lowest
	}
	
	// up is one past hi, so get pointer to hi (could be tmax or around tmax)
	auto hi = imuSeq.begin() + (up - imuSeq.begin() - 1);
	if (abs(hi.first - tmax) > eps) // tmax does not match hi.first
	{
		// adjust forward iterators to set up interpolation
		auto hiPrev = hi;
		auto hiNext = hi;
		if (hi.first < tmax) ++hiNext;
		else if (up != imuSeq.end()) hiPrev = imuSeq.begin() + (hi - imuSeq.begin() - 1);

		// preform interpolation
		auto ui = interp(tmax, hiPrev.first, hiNext.first - hiPrev.first,
						 hiPrev.second, hiNext.second - hiPrev.second, eps);

		if (hi.first > tmax) useq.back() = {tmax, ui}; // overwrite last pos with tmax to make it the highest
		else useq.emplace_back({tmax, ui}); // add tmax to back to make it the highest
	}
	
	return useq;
}
