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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<cstdlib>
#include <stdio.h>
#include <sys/stat.h>

#include<opencv2/core/core.hpp>

#include"System.h"

//#include "Osmap.h"
//#include"../../include/Osmap.h"   /// for saving the map

/// run with:
// ./Examples/Monocular/validate_black_fly Vocabulary/ORBvoc.txt Examples/Monocular/black_fly_downsized.yaml /mnt/data/input/validation_data/test_flight_run_1_blackflys_1_0/

using namespace cv;
using namespace std;

void LoadImages(const string &strSequence, vector<int> &vFrameNums, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_black_fly path_to_vocabulary path_to_settings path_to_sequence [n_start] [n_end]" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<int> vFrameNums;
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

    string dataPath = string(argv[3]);
    LoadImages(dataPath, vFrameNums, vstrImageFilenames, vTimestamps);

    string outputPath = dataPath;
    outputPath.append("/output");
    mkdir(outputPath.c_str(), 0777);

    int nTrackResets = 0;
    int nImages = vstrImageFilenames.size();

    int nStart = 0;
    int nEnd = nImages;
    int nStep = 1;

    if (argc >= 5) {
      nStart = atoi(argv[4]);
    }

    if (argc == 6) {
      nEnd = atoi(argv[5]);

      if (nEnd > nImages) {
        nEnd = nImages;
      }
    }

    if (argc == 7) {
      nStep = atoi(argv[6]);
    }



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,outputPath,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "Start Image: " << nStart << endl;
    cout << "End Image: " << nEnd << endl << endl;


    cv::Mat mTcw;

    // Main loop
    cv::Mat im;
    for(int ni=nStart; ni<nEnd; ni+=nStep)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        int nframe = vFrameNums[ni];

        if (SLAM.mpTracker->mState != SLAM.mpTracker->mLastProcessedState) {
          cout << "Processing Image: " << ni << " ("<<vstrImageFilenames[ni] << ") @ time " << tframe << endl;
          cout << SLAM.mpTracker->mLastProcessedState << " --> " << SLAM.mpTracker->mState << endl;

          if (SLAM.mpTracker->mState == 3) {
            
            string outputPath = dataPath;
            outputPath.append("/output");
            SLAM.SaveKeyFrameTrajectoryTUM("/mnt/data/output/KeyFrameTrajectory_" + std::to_string(nTrackResets) + ".txt");
            SLAM.SaveTrajectoryTUM("/mnt/data/output/CameraTrajectory_" + std::to_string(nTrackResets) + ".txt");
            SLAM.mpTracker->Reset();
            nTrackResets ++;
          }
        }

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif


        // Pass the image to the SLAM system

        mTcw = SLAM.TrackMonocular(im,nframe,tframe);



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // save the map
    // ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);
    // osmap.mapSave("airsim_map");	// "myFirstMap" or "myFirstMap.yaml", same thing
    // save the pointcloud
    // SLAM.CreatePCD("pointcloud.pcd");

    // Stop all threads
    SLAM.Shutdown();



    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/mnt/data/output/KeyFrameTrajectory_" + std::to_string(nTrackResets) + ".txt");
    SLAM.SaveTrajectoryTUM("/mnt/data/output/CameraTrajectory_" + std::to_string(nTrackResets) + ".txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<int> &vFrameNums, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/frame_info.txt";
    fTimes.open(strPathTimeFile.c_str());

    string strPrefixLeft = strPathToSequence;

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            istringstream iss(s);

            vector<string> split_line((istream_iterator<string>(iss)),
                                 istream_iterator<string>());


            // save frame number
            int n;
            n = atoi(split_line[0].c_str());
            vFrameNums.push_back(n);

            // save filename
            stringstream ss;
            ss << split_line[1];
            vstrImageFilenames.push_back(strPrefixLeft + "/" + ss.str());

            // save frame time
            double t;
            t = atof(split_line[2].c_str());
            vTimestamps.push_back(t);
        }
    }

    // string strPrefixLeft = strPathToSequence + "/images_slam/frame_";

    // const int nTimes = vTimestamps.size();
    // vstrImageFilenames.resize(nTimes);

    // for(int i=0; i<nTimes; i++)
    // {
    //     stringstream ss;
    //     ss << setfill('0') << setw(5) << i;
    //     vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".jpg";
    // }
}
