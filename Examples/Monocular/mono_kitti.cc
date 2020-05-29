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

#include <fmt/format.h>
#include <glog/logging.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "System.h"

using namespace std;
using namespace fmt;

void LoadImages(const string& pathToSequence, vector<string>& imageFilenames, vector<double>& timestamps) {
    LOG(INFO) << format("Load images from KITTI folder {}", pathToSequence);

    // read timestamp
    ifstream fTimes;
    string strPathTimeFile = pathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            timestamps.push_back(t);
        }
    }

    // parse image files
    string strPrefixLeft = pathToSequence + "/image_0/";
    const int nTimes = timestamps.size();
    imageFilenames.resize(nTimes);
    for (int i = 0; i < nTimes; ++i) {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        imageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    if (argc != 4) {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> imageFilenames;
    vector<double> timestamps;
    LoadImages(string(argv[3]), imageFilenames, timestamps);

    const size_t kImageNum = imageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> timesTrack;
    timesTrack.resize(kImageNum);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << format("Images in the sequence: {}", kImageNum) << endl << endl;

    // Main loop
    cv::Mat im;
    for (size_t n = 0; n < kImageNum; ++n) {
        // Read image from file
        im = cv::imread(imageFilenames[n], cv::IMREAD_UNCHANGED);
        double tframe = timestamps[n];

        if (im.empty()) {
            cerr << endl << "Failed to load image at: " << imageFilenames[n] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        timesTrack[n] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (n < kImageNum - 1)
            T = timestamps[n + 1] - tframe;
        else if (n > 0)
            T = tframe - timestamps[n - 1];

        if (ttrack < T) usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(timesTrack.begin(), timesTrack.end());
    float totaltime = 0;
    for (size_t n = 0; n < kImageNum; ++n) {
        totaltime += timesTrack[n];
    }
    cout << "-------" << endl << endl;
    cout << format("median tracking time: {}", timesTrack[kImageNum / 2]) << endl;
    cout << format("mean tracking time: {}", totaltime / kImageNum) << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    google::ShutdownGoogleLogging();
    return 0;
}
