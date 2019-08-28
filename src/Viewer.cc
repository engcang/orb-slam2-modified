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

#include "Viewer.h"
#include <mutex>

namespace ORB_SLAM2
{
    Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, Tracking *pTracking, const string &strSettingPath):mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpTracker(pTracking)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if(fps<1)
            fps=30;
        mT = 1e3/fps;
        ShowTrack = fSettings["ShowTrack"];

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if(mImageWidth<1 || mImageHeight<1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }
    }

    void Viewer::Run()
    {
        if(ShowTrack)
        {
            cv::namedWindow("ORB-SLAM2: Current Frame");
            while(1)
            {   cv::Mat im = mpFrameDrawer->DrawFrame();
                cv::imshow("ORB-SLAM2: Current Frame",im);
                cv::waitKey(mT);}
        }
    }
}
