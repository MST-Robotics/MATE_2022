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
    void Process(Mat &m_pFrame, Mat &m_pFinalImg, int &m_nTargetCenterX, int &m_nTargetCenterY, double &m_dContrastValue, double &m_dTargetAngle, bool &m_bTuningMode, bool &m_bDrivingMode, bool &m_bTrackingMode, bool &m_bSolvePNPEnabled, vector<int> &m_vTrackbarValues, vector<double> &m_vSolvePNPValues, VideoGet &pVideoGetter, shared_timed_mutex &m_pMutexGet, shared_timed_mutex &m_pMutexShow);
    int SignNum(double dVal);
    vector<double> SolveObjectPose(vector<Point2f> m_pImagePoints, Mat &m_pFinalImg, Mat &m_pFrame, int nTargetPositionX, int nTargetPositionY);
    void SetIsStopping(bool bIsStopping);
    bool GetIsStopped();
    int GetFPS();

private:
    // Declare class objects and variables.
    Mat							m_pHSVImg;
    Mat							m_pBlurImg;
    Mat							m_pFilterImg;
    Mat							m_pDilateImg;
    Mat							m_pCorners;
    Mat							m_pCornersNormalized;
    Mat							m_pCornersScaled;
    Mat							m_pKernel;
    Mat							m_pCameraMatrix;
    Mat							m_pDistanceCoefficients;
    vector<Point3f>				m_pObjectPoints;
    vector<vector<Point>>		m_vContours;
    vector<Vec4i>				m_pHierarchy;
    FPS*						m_pFPS;

    int							m_nFPS;
    int							m_nMinimumNumberOfTapeVertices;
    int							m_nMinimumPowerCellRadius;
    int							m_nMaxContours;
    int							m_nScreenHeight;
    int							m_nScreenWidth;
    int							m_nGreenBlurRadius;
    int							m_nOrangeBlurRadius;
    int							m_nHorizontalAspect;
    int							m_nVerticalAspect;
    int							m_nNumberOfPolyCorners;
    double						m_dPipeLargestContourArea;
    double						m_dCameraFOV;
    double						m_dFocalLength;
    const double				PI = 3.14159265358979323846;
    bool						m_bIsStopping;
    bool						m_bIsStopped;
};
///////////////////////////////////////////////////////////////////////////////
#endif