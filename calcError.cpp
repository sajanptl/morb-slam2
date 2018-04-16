#include <fstream>
#include <iostream>
// #include <cmath>
#include <math.h>

using namespace std;

int main(int argc, char const *argv[])
{
	ifstream baseline("CameraTrajectory.txt");
	ifstream ours("TestOursCameraTrajectory.txt");
	ofstream error("errors.txt");
	double r11, r12, r13, t14, r21, r22, r23, t24, r31, r32, r33, t34;
	double r11b, r12b, r13b, t14b, r21b, r22b, r23b, t24b, r31b, r32b, r33b, t34b;

	double total = 0;
	double totalR = 0;
	double num = 0;
	while (baseline >> r11 >> r12 >> r13 >> t14 >> r21 >> r22 >> r23 >> t24 >> r31 >> r32 >> r33 >> t34 &&
			ours >> r11b >> r12b >> r13b >> t14b >> r21b >> r22b >> r23b >> t24b >> r31b >> r32b >> r33b >> t34b) {
		double xe = t14 + t14b;
		double ye = t24 + t24b;
		double ze = t34 + t34b;
		double sqrtE = sqrt(xe * xe + ye * ye + ze * ze);
		double d = 0.5 * (xe + ye + ze - 1.0);
  		double rotE = acos(max(min(d, 1.0), -1.0));
		error << xe << " " << ye << " " << ye << " " << sqrtE << "\n";
		total += sqrtE;
		totalR += rotE;
		num++;
	}
	error << total / num <<"\n";
	error << totalR / num << "\n";
	return 0;
}