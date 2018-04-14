/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include <System.h>

// STL aliasing
using std::vector;
using std::string;
using std::sort;
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::getline;
using std::stringstream;
using std::setfill;
using std::setw;
using std::to_string;

// STL chrono aliasing
using namespace std::chrono;
using chrono::duration;
using chrono::duration_cast;

#ifdef COMPILEDWITHC11
    using myClock = std::chrono::steady_clock;
#else
    using myClock = std::chrono::monotonic_clock;
#endif

// OpenCV aliasing
using cv::Mat;
using cv::imread;

// ORB_SLAM2 aliasing
using ORB_SLAM2::System;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
/**
 * @brief Loads in the filenames of images and timestamps for a given sequence.
 */
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<string> &imuLines);

string generateIMUFilename(int n);

Eigen::VectorXd parseIMULine(string imuLine);

double timestampsLineToDouble(string line);

/**
 * @brief Main driver for the Stereo Kitti experiment.
 */
int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<string> imuLines;
    vector<double> vTimestamps;
    vector<double> iTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps, imuLines);

    const size_t nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    Mat imLeft, imRight;
    for (size_t ni = 0; ni < nImages; ++ni)
    {
        // Read left and right images from file
        imLeft  = imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imRight = imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
            return 1;
        }
        
        myClock::time_point t1 = myClock::now();
        // TODO: Read in IMU/Oxts file and call System.AddIMUMeasurement()
        SLAM.AddIMUMeasurement(parseIMULine(imuLines[ni]), vTimestamps[ni]);
        // END TODO
		
		// Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe);
	
        myClock::time_point t2 = myClock::now();
        
		double ttrack = duration_cast<duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < (nImages - 1)) T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0) 		T = tframe - vTimestamps[ni - 1];

        if (ttrack < T) usleep((T - ttrack) * 1e6);
    }
	
	cout << "Shutting down SLAM threads" << endl;

    // Stop all threads
    SLAM.Shutdown();
	
	cout << "Accumulating timing info" << endl;

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (size_t ni = 0; ni < nImages; ++ni) totaltime += vTimesTrack[ni];

    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("TestOursCameraTrajectory.txt");
	
	cout << "Done!" << endl;

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<string> &imuLines)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }


    string strPrefixLeft  = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const size_t nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    imuLines.resize(nTimes);
    for (size_t i=0; i < nTimes; ++i)
    {
        ifstream imuFile;
        string strPathIMUFile = strPathToSequence + "oxts/data/" + generateIMUFilename(i);
        imuFile.open(strPathIMUFile.c_str());
        if (!getline(imuFile, imuLines[i])) {
            cout << "IMU reading failed at index " << i << "\n";
        }
        stringstream ss;
        ss << setfill('0') << setw(6) << to_string(i);
        vstrImageLeft[i]  = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

string generateIMUFilename(int n) {
    stringstream IMUFilename;
    IMUFilename << setfill('0') << setw(10) << n;
    return IMUFilename.str() + ".txt";
}

Eigen::VectorXd parseIMULine(string imuLine) {
    Eigen::VectorXd imu(6);
    stringstream ss(imuLine);
    int counter = 0;
    double datum;
    while (ss >> datum) {
        // ax = 11, ay = 12, az = 13
        // wx = 17, wy = 18, wz = 19
        if (counter == 14) {
            imu[0] = datum;
        }
        else if (counter == 15) {
            imu[1] = datum;
        }
        else if (counter == 16) {
            imu[2] = datum;
        }
        else if (counter == 20) {
            imu[3] = datum;
        }
        else if (counter == 21) {
            imu[4] = datum;
        }
        else if (counter == 22) {
            imu[5] = datum;
        }
        ++counter;
    }
    return imu;
}

double timestampsLineToDouble(string line) {
    double year, month, day, hours, minutes, seconds;
    sscanf(line.c_str(), "%lf-%lf-%lf %lf:%lf:%lf", &year, &month, &day, &hours, &minutes, &seconds);
    // Assumption: We do not cross over a month or a year, only consider days because there are fewer excepions in computation
    return (day * 86400) + (hours * 3600) + (minutes * 60) + seconds;
}
