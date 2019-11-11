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

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathTimes, vector<string> &vstrImageLeft, vector<double> &vTimeStamp);

int main(int argc, char **argv)
{
    if(argc != 6 && argc !=7)
    {
        cerr << endl << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings path_to_left_folder path_to_right_folder path_to_times_file" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    vector<double> vTimeStamp2;

    LoadImages(string(argv[3]), string(argv[5]), vstrImageLeft, vTimeStamp);
    LoadImages(string(argv[4]), string(argv[6]), vstrImageRight, vTimeStamp2);

    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    const int nImages = (vstrImageLeft.size()>vstrImageRight.size())?vstrImageRight.size():vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    cv::Mat imLeft, imRight;
    cv::Mat Tcw;

    int ShowTrack = fsSettings["ShowTrack"]; // viewer removed
    //cv::Mat imtemp;
    //cv::Mat immask = cv::imread( string(argv[3])+"/mask.bmp", CV_LOAD_IMAGE_UNCHANGED );         // used for Oxford car dataset
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }
        double tframe = vTimeStamp[ni];


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        Tcw = SLAM.TrackStereo(imLeft,imRight,tframe);
        //SLAM.TrackMonocular(im,tframe/1e6);

        if (Tcw.size().height>0 && Tcw.size().width >0){
            cv::Mat rotation(3,3,CV_32F);
            cv::Mat translation(3,1,CV_32F);

            rotation = Tcw.rowRange(0,3).colRange(0,3).t();
            translation = -rotation*Tcw.rowRange(0,3).col(3);
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(rotation);

            cout << "E,D,N " << translation.at<float>(0) << " " << translation.at<float>(1) << " " << translation.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
            } 
        if(ShowTrack) // Extracted from viewer
        {   cv::namedWindow("ORB-SLAM2: Current Frame");
            cv::Mat im = SLAM.getimage();
            cv::imshow("ORB-SLAM2: Current Frame",im);
            cv::waitKey(1);}

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
            T = vTimeStamp[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimeStamp[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)/1e3);
        //    usleep(T-ttrack);
        //usleep(2e5);
    }
    //cv::waitKey();
    
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
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<double> &vTimeStamp)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamp.reserve(5000);
    vstrImageLeft.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamp.push_back(t/1e9);
        }
    }
}

// void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamp)
// {
//     ifstream f;
//     f.open(strFile.c_str());

//     // skip first three lines
//     /*string s0;
//     getline(f,s0);
//     getline(f,s0);
//     getline(f,s0);*/

//     while(!f.eof())
//     {
//         string s;
//         getline(f,s);
//         if(!s.empty())
//         {
//             stringstream ss;
//             ss << s;
//             double t;
//             string sRGB;
//             ss >> t;
//             vTimestamp.push_back(t);
//             ss >> sRGB;
//             vstrImageFilenames.push_back(sRGB);
//         }
//     }
// }
