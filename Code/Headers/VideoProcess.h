/****************************************************************************
			Description:	Defines the VideoProcess Class

			Classes:		VideoProcess

			Project:		MATE 2022

			Copyright 2021 MST Design Team - Underwater Robotics.
****************************************************************************/
#ifndef VideoProcess_h
#define VideoProcess_h

#include <cstdio>
#include <string>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>

#include "VideoGet.h"
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


class VideoProcess
{
public:
    // Declare class methods.
    VideoProcess();
    ~VideoProcess();
    void Process(Mat &frame, Mat &finalImg, int &targetCenterX, int &targetCenterY, int &centerLineTolerance, double &contourAreaMinLimit, double &contourAreaMaxLimit, bool &tuningMode, bool &drivingMode, bool &trackingMode, bool &solvePNPEnabled, vector<int> &trackbarValues, vector<double> &solvePNPValues, VideoGet &VideoGetter, shared_timed_mutex &MutexGet, shared_timed_mutex &MutexShow);
    int SignNum(double val);
    vector<double> SolveObjectPose(vector<Point2f> imagePoints, Mat &finalImg, Mat &frame, int targetPositionX, int targetPositionY);
    void SetIsStopping(bool isStopping);
    bool GetIsStopped();
    int GetFPS();

private:
    // Declare class objects and variables.
    Mat							HSVImg;
    Mat							blurImg;
    Mat							filterImg;
    Mat							dilateImg;
    Mat							corners;
    Mat							cornersNormalized;
    Mat							cornersScaled;
    Mat							kernel;
    Mat							cameraMatrix;
    Mat							distanceCoefficients;
    vector<Point3f>				objectPoints;
    vector<vector<Point>>		contours;
    vector<Vec4i>				hierarchy;
    FPS*						FPSCounter;

    int							FPSCount;
    int							screenHeight;
    int							screenWidth;
    int							greenBlurRadius;
    int							horizontalAspect;
    int							verticalAspect;
    double						cameraFOV;
    double						focalLength;
    const double				PI = 3.14159265358979323846;
    bool						isStopping;
    bool						isStopped;
};
///////////////////////////////////////////////////////////////////////////////
#endif