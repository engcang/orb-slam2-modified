/**
* This file is part of CubemapSLAM.
*
* Copyright (C) 2017-2019 Yahui Wang <nkwangyh at mail dot nankai dot edu dot cn> (Nankai University)
* For more information see <https://github.com/nkwangyh/CubemapSLAM>
*
* CubemapSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CubemapSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CubemapSLAM. If not, see <http://www.gnu.org/licenses/>.
*/

/*
* CubemapSLAM is based on ORB-SLAM2 and Multicol-SLAM which were also released under GPLv3
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* and <https://github.com/urbste/MultiCol-SLAM>
* Steffen Urban <urbste at googlemail.com>
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cassert>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include "System.h"
#include "CamModelGeneral.h"
#include "Frame.h"
#include "ORBExtractor.h"

unsigned int frame_counter;

// void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
// {
//     ifstream f;
//     f.open(strFile.c_str());
//     int ts = 0;
//     while(!f.eof())
//     {
//         string s;
//         getline(f,s);
//         if(!s.empty())
//         {
//             stringstream ss;
//             ss << s;
//             string sRGB;
//             vTimestamps.push_back(ts);
//             ts++;
//             ss >> sRGB;
//             vstrImageFilenames.push_back(sRGB);
//         }
//     }
// }

void LoadImages(const string &strPathLeft, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
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
            vTimeStamps.push_back(t/1e9);
        }
    }
}
int main(int argc, char **argv)
{
    frame_counter = 0;
    //read in params
    std::string vocabularyFilePath(argv[1]);
    std::string settingFilePath(argv[2]);
    System cubemapSLAM(vocabularyFilePath, settingFilePath.c_str());
    
    //get image list
    std::string fisheyeImgPath(argv[3]);
    if(fisheyeImgPath[fisheyeImgPath.length()-1] != '/')
        fisheyeImgPath += "/";
    std::string fisheyeImgListPath(argv[4]);
    std::string cubemapMaskPath(argv[5]);
    std::string trajSavingPath(argv[6]);
    std::string perfSavingPath(argv[7]);

    vector<string> vstrImage;
    vector<double> vTimestamps;

    LoadImages(fisheyeImgPath, fisheyeImgListPath, vstrImage, vTimestamps);
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    int ShowTrack = fsSettings["ShowTrack"]; // viewer removed
    cv::Mat Tcw; //result


    if(vstrImage.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }

    cv::Mat cubemapMask = cv::imread(cubemapMaskPath.c_str(), cv::IMREAD_GRAYSCALE);
    //when image mask is not needed, a grayscale image with all pixel as 255 will be created
    if(cubemapMask.empty())
    {
        std::cout << "fail to read the mask: " << cubemapMaskPath << std::endl;
    }

    // std::ifstream fin(fisheyeImgListPath);
    // std::vector<std::string> fisheyeImgNames;
    // std::vector<double> vTimestamps;
    // std::string line;
    // while(std::getline(fin, line))
    // {
    //     std::stringstream ss(line);
    //     double ts;
    //     ss >> ts;
    //     std::string name;
    //     ss >> name;
    //     size_t p = name.find_last_of("/");
    //     name = name.substr(p+1, name.length());
        
    //     vTimestamps.push_back(ts);
    //     fisheyeImgNames.push_back(name); 
    // }

    int offset = 0;
    int width = CamModelGeneral::GetCamera()->GetCubeFaceWidth(), height = CamModelGeneral::GetCamera()->GetCubeFaceHeight();
    cv::Mat cubemapImg(height * 3, width * 3, CV_8U, cv::Scalar::all(0));
    cv::Mat cubemapImg_front = cubemapImg.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat cubemapImg_left = cubemapImg.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat cubemapImg_right = cubemapImg.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat cubemapImg_upper = cubemapImg.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat cubemapImg_lower = cubemapImg.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);
    
    const int imageCnt = vstrImage.size();
    std::cout << "find " << imageCnt << " images" << std::endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(imageCnt);

    std::vector<cv::Mat> origImages;

    int nStep = 1;
    for(int idx = 0; idx < imageCnt; ++idx)
    {
        if(idx % nStep != 0)
            continue;
        //read in images
        cv::Mat fisheyeImg = cv::imread(vstrImage[idx], cv::IMREAD_GRAYSCALE);

        std::cout << "reading image " << vstrImage[idx] << std::endl;
        if(!fisheyeImg.data)
        {
            std::cout << "fail to read image in " << vstrImage[idx] << std::endl;
            exit(EXIT_FAILURE);
        }

        cubemapSLAM.CvtFisheyeToCubeMap_reverseQuery_withInterpolation(cubemapImg, fisheyeImg, cv::INTER_LINEAR);
        
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        //track cubemap
        Tcw = cubemapSLAM.TrackCubemap(cubemapImg, cubemapMask, vTimestamps[idx]); 

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        if (Tcw.size().height>0 && Tcw.size().width >0){
            cv::Mat rotation(3,3,CV_32F);
            cv::Mat translation(3,1,CV_32F);

            rotation = Tcw.rowRange(0,3).colRange(0,3).t();
            translation = -rotation*Tcw.rowRange(0,3).col(3);
            vector<float> q = Converter::toQuaternion(rotation);
            std::cout << "E,D,N " << translation.at<float>(0) << " " << translation.at<float>(1) << " " << translation.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        if(ShowTrack) // Extracted from viewer
        {   cv::namedWindow("CubeMap-SLAM: Current Frame");
            cv::Mat im = cubemapSLAM.getimage();
            cv::imshow("CubeMap-SLAM: Current Frame",im);
            cv::waitKey(1);}

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[idx]=ttrack;
    }
    
    // Stop all threads
    cubemapSLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int idx=0; idx<imageCnt; idx++)
    {
        totaltime+=vTimesTrack[idx];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[imageCnt/2] << endl;
    cout << "mean tracking time: " << totaltime/imageCnt << endl;

    std::ofstream f;
    if(trajSavingPath.empty())
        trajSavingPath = std::string("KeyFrameTrajectory.txt");
    f.open(perfSavingPath.c_str());
    f << fixed;
    f << "-------" << endl << endl;
    f << "median tracking time: " << vTimesTrack[imageCnt/2] << endl;
    f << "mean tracking time: " << totaltime/imageCnt << endl;
    f << "tracking frames/ total frames: " << frame_counter << "/ " << imageCnt << " " << static_cast<float>(frame_counter)/imageCnt << std::endl;
    f.close();

    // Save camera trajectory
    cubemapSLAM.SaveKeyFrameTrajectoryTUM(trajSavingPath);
}
