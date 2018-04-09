#ifndef IMUSEQUENCE_H
#define IMUSEQUENCE_H

#include <utility>
#include <deque>
#include <mutex>
#include <Eigen/Core>

namespace ORB_SLAM2
{

class IMUSequence
{
public:
	IMUSequence() : maxSeqLen(-1) {}
	IMUSequence(int maxLen) : maxSeqLen(maxLen) {}
	
	void add(const Eigen::VectorXd& u, double t);

	size_t size() { std::lock_guard<std::mutex> lck(mtx); return imuSeq.size(); } 

	void reset() { std::lock_guard<std::mutex> lck(mtx); imuSeq.clear(); }
	
	std::deque<std::pair<double, Eigen::VectorXd>> get(double tmin, double tmax);

private:
	int maxSeqLen;
	std::deque<std::pair<double, Eigen::VectorXd>> imuSeq;
	std::mutex mtx;
};

} // close ORB_SLAM2 namespace

#endif // IMUSEQUENCE_H
