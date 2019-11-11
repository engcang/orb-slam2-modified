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


#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "CamModelGeneral.h"
// #include "Viewer.h"
// #include "MapDrawer.h"
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include <unistd.h> // added for 18.04, to use usleep
#include "Converter.h"


class Tracking;
class Map;
class LocalMapping;
class LoopClosing;
// class Viewer;
// class MapDrawer;
class FrameDrawer;

class System
{
public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    // System(const string &strVocFile, const std::string &strSettingsFile, const bool bUseViewer=true);
    System(const string &strVocFile, const std::string &strSettingsFile);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackCubemap(const cv::Mat &im, const cv::Mat &mask, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();
    
    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();

    //convert fisheye image to cubemap
    void CvtFisheyeToCubeMap_reverseQuery(cv::Mat &cubemapImg, const cv::Mat &fisheyeImg);
    //creating mapping
    void CreateUndistortRectifyMap();
    //convert fisheye image to cubemap
    void CvtFisheyeToCubeMap_reverseQuery_withInterpolation(cv::Mat &cubemapImg, const cv::Mat &fisheyeImg, 
        int interpolation, int borderType=cv::BORDER_CONSTANT, const cv::Scalar& borderValue=cv::Scalar());
    //convert fisheye to cubemap with pCamModel->FisheyeToCubemap
    void CvtFisheyeToCubeMap(cv::Mat &cubemapImg, const cv::Mat &fisheyeImg);

    // Whether with mask
    int mnWithFisheyeMask;
    cv::Mat getimage(); //added to replace viewer

private:
    
    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;
    
    // The viewer draws the map and the current camera pose. It uses Pangolin.
    // Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    // MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    // std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPoints;
    std::mutex mMutexState;

    // image Mapping
    cv::Mat mMap1;
    cv::Mat mMap2;
};

#endif // SYSTEM_H
