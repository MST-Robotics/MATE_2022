/****************************************************************************
			Description:	Defines the VideoShow Class

			Classes:		VideoShow

			Project:		MATE 2022

			Copyright 2021 MST Design Team - Underwater Robotics.
****************************************************************************/
#ifndef VideoShow_h
#define VideoShow_h

#include <cstdio>
#include <string>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <iostream>
#include <algorithm>
#include <vector>

#include "FPS.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>
#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include <cameraserver/CameraServer.h>

using namespace cv;
using namespace cs;
using namespace nt;
using namespace frc;
using namespace wpi;
using namespace std;
///////////////////////////////////////////////////////////////////////////////

class VideoShow
{
public:
    // Define class methods.
    VideoShow();
    ~VideoShow();
    void ShowFrame(Mat &frame, vector<CvSource> &cameraSources, shared_timed_mutex &Mutex);
    void SetIsStopping(bool isStopping);
    bool GetIsStopped();
    int GetFPS();

private:
    // Declare class objects and variables.
    FPS*						FPSCounter;
    
    int							FPSCount;
    bool						isStopping;
    bool						isStopped;
};
///////////////////////////////////////////////////////////////////////////////
#endif