#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>
#include <Converter.h>
#include <Utils.hpp>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathTimes, vector<string> &vstrImageLeft, vector<double> &vTimeStamps);

double hz;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#else
#define SET_CLOCK(t0) \
        std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif

#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

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

    // if(vstrImageLeft.size()!=vstrImageRight.size())
    // {
    //     cerr << "ERROR: Different number of left and right images." << endl;
    //     return 1;
    // }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    bool Rectification;
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, M1l,M2l,M1r,M2r;
    fsSettings["Rectification"] >> Rectification;
    if(Rectification){
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }


    const int nImages = (vstrImageLeft.size()>vstrImageRight.size())?vstrImageRight.size():vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Main loop
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    cv::Mat Tcw;

    int ShowTrack = fsSettings["ShowTrack"]; // viewer removed

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

        if(Rectification){
            cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
            cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);}
        double tframe = vTimeStamp[ni];


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        if(Rectification)
            Tcw = SLAM.TrackStereo(imLeftRect,imRightRect,tframe);
        else
            Tcw = SLAM.TrackStereo(imLeft,imRight,tframe);

        if (Tcw.size().height>0 && Tcw.size().width >0){
            cv::Mat rotation(3,3,CV_32F);
            cv::Mat translation(3,1,CV_32F);

            rotation = Tcw.rowRange(0,3).colRange(0,3).t();
            translation = -rotation*Tcw.rowRange(0,3).col(3);
            vector<float> q = ORB_SLAM2::Converter::toQuaternion(rotation);
            }
            //cout << "E,D,N " << translation.at<float>(0) << " " << translation.at<float>(1) << " " << translation.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl; }
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
        cout << 1/ttrack << endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimeStamp[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimeStamp[ni-1];

//        if(ttrack<T)
//            usleep((T-ttrack)*1e6);
    }
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
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}

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
