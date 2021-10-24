/****************************************************************************
			Description:	Defines the VideoGet Class

			Classes:		VideoGet

			Project:		2020 DeepSpace Vision Code

			Copyright 2020 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef VideoGet_h
#define VideoGet_h

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

using namespace std;
using namespace cv;
///////////////////////////////////////////////////////////////////////////////


class VideoGet
{
public:
    // Declare class methods.
    VideoGet();
    ~VideoGet();
    void StartCapture(Mat &m_pFrame, bool &bCameraSourceIndex, bool &bDrivingMode, shared_timed_mutex &m_pMutex);
    void SetIsStopping(bool bIsStopping);
    bool GetIsStopped();
    int GetFPS();

private:
    // Declare class objects and variables.
    FPS*					m_pFPS;
    VideoCapture			cap;
    
    int						m_nFPS;
    bool					m_bIsStopping;
    bool					m_bIsStopped;
};
///////////////////////////////////////////////////////////////////////////////
#endif