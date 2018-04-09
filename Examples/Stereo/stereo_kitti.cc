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

/**
 * @brief Loads in the filenames of images and timestamps for a given sequence.
 */
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

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
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

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
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
	
	cout << "Done!" << endl;

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
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
    for (size_t i=0; i < nTimes; ++i)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << to_string(i);
        vstrImageLeft[i]  = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
