/****************************************************************************
			Description:	Implements the VideoProcess Class

			Classes:		VideoProcess

			Project:		MATE 2022

			Copyright 2021 MST Design Team - Underwater Robotics.
****************************************************************************/
#include "../Headers/VideoProcess.h"
///////////////////////////////////////////////////////////////////////////////


/****************************************************************************
        Description:	VideoProcess constructor.

        Arguments:		None

        Derived From:	Nothing
****************************************************************************/
VideoProcess::VideoProcess()
{
    // Create object pointers.
    m_pFPS									= new FPS();
    
    // Initialize member variables.
    m_pKernel								= getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    m_nFPS									= 0;
    m_nNumberOfPolyCorners					= 8;
    m_nMinimumNumberOfTapeVertices			= 4;
    m_nMinimumPowerCellRadius				= 1;
    m_nMaxContours							= 4;
    m_nGreenBlurRadius						= 3;
    m_nOrangeBlurRadius						= 3;
    m_nHorizontalAspect						= 4;
    m_nVerticalAspect						= 3;
    m_dPipeLargestContourArea				= 2000;
    m_dCameraFOV							= 75;
    m_nScreenWidth							= 640;
    m_nScreenHeight							= 480;
    m_dFocalLength							= (m_nScreenWidth / 2.0) / tan((m_dCameraFOV * PI / 180.0) / 2.0);
    m_bIsStopping							= false;
    m_bIsStopped							= false;

    ////
    // Setup SolvePNP data.
    ////

    // Reference object points.
    m_pObjectPoints.emplace_back(Point3f(39.50, 0.0, 0.0));
    m_pObjectPoints.emplace_back(Point3f(29.50, -17.0, 0.0));
    m_pObjectPoints.emplace_back(Point3f(9.75, -17.0, 0.0));
    m_pObjectPoints.emplace_back(Point3f(0.0, 0.0, 0.0));

    // Precalibrated camera matrix values.
    double mtx[3][3] = {{516.5613698781304, 0.0, 320.38297194779585},		//// PSEye Cam
                        {0.0, 515.9356734667019, 231.73585601568368},
                        {0.0, 0.0, 1.0}};
    // double mtx[3][3] = {{659.3851992714341, 0.0, 306.98918779442675},		//// Lifecam
    // 					{0.0, 659.212123568372, 232.07157473243464},
    // 					{0.0, 0.0, 1.0}};
    m_pCameraMatrix = Mat(3, 3, CV_64FC1, mtx).clone();

    // Precalibration distance/distortion values.
    double dist[5] = {-0.0841024904469607, 0.014864043816324026, -0.00013887041018197853, -0.0014661216967276468, 0.5671907234987197};	//// PSEye Cam
    // double dist[5] = {0.1715327237204972, -1.3255106761114646, 7.713495040297368e-07, -0.0035865453000784634, 2.599132082766894};	//// Lifecam
    m_pDistanceCoefficients = Mat(1, 5, CV_64FC1, dist).clone();
}

/****************************************************************************
        Description:	VideoProcess destructor.

        Arguments:		None

        Derived From:	Nothing
****************************************************************************/
VideoProcess::~VideoProcess()
{
    // Delete object pointers.
    delete m_pFPS;

    // Set object pointers as nullptrs.
    m_pFPS = nullptr;
}

/****************************************************************************
        Description:	Processes frames with OpenCV.

        Arguments(dear god help us): MAT&, MAT&, INT&, INT&, DOUBLE&, DOUBLE&, BOOL&, BOOL&, BOOL&, BOOL&, VECTOR<INT>, VECTOR<DOUBLE>, VIDEOGET, SHARED_TIMED_MUTEX&, SHARED_TIMED_MUTEX&

        Returns: 		Nothing
****************************************************************************/
void VideoProcess::Process(Mat &m_pFrame, Mat &m_pFinalImg, int &m_nTargetCenterX, int &m_nTargetCenterY, double &m_dContrastValue, double &m_dTargetAngle, bool &m_bTuningMode, bool &m_bDrivingMode, bool &m_bTrackingMode, bool &m_bSolvePNPEnabled, vector<int> &m_vTrackbarValues, vector<double> &m_vSolvePNPValues, VideoGet &pVideoGetter, shared_timed_mutex &m_pMutexGet, shared_timed_mutex &m_pMutexShow)
{
    // Give other threads enough time to start before processing camera frames.
    this_thread::sleep_for(std::chrono::milliseconds(800));

    while (1)
    {
        // Increment FPS counter.
        m_pFPS->Increment();

        // Make sure frame is not corrupt.
        try
        {
            // Acquire resource lock for read thread. NOTE: This line has been commented out to improve processing speed. VideoGet takes to long with the resources.
            // shared_lock<shared_timed_mutex> guard(m_pMutexGet);

            if (!m_pFrame.empty())
            {
                // Convert image from RGB to HSV.
                //cvtColor(m_pFrame, m_pHSVImg, COLOR_BGR2HSV);
                // Acquire resource lock for show thread only after m_pFrame has been used.
                unique_lock<shared_timed_mutex> guard(m_pMutexShow);
                // Copy frame to a new mat.
                m_pFinalImg = m_pFrame.clone();
                // Blur the image.
                blur(m_pFrame, m_pBlurImg, Size(m_nGreenBlurRadius, m_nGreenBlurRadius));
                // Filter out specific color in image.
                inRange(m_pBlurImg, Scalar(m_vTrackbarValues[0], m_vTrackbarValues[2], m_vTrackbarValues[4]), Scalar(m_vTrackbarValues[1], m_vTrackbarValues[3], m_vTrackbarValues[5]), m_pFilterImg);
                // Apply blur to image.
                dilate(m_pFilterImg, m_pDilateImg, m_pKernel);

                // Find countours of image.
                findContours(m_pDilateImg, m_vContours, m_pHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);		////RETR_TREE //// TRY CHAIN_APPROX_SIMPLE		//// Not sure what this method of detection does, but it worked before: CHAIN_APPROX_TC89_KCOS

                // Driving mode.
                if (!m_bDrivingMode)
                {
                    // Tracking mode. (Pipe or Tape)
                    if (m_bTrackingMode)
                    {
                        /****************************************************
                        *			Track pipe target
                        *****************************************************/

                        // Draw all contours in white.
                        // drawContours(m_pFinalImg, m_vContours, -1, Scalar(255, 255, 210), 1, LINE_4, m_pHierarchy);

                        if (m_vContours.size() >= 2)
                        {
                            // 'Round off' all contours with convexHull.
                            vector<vector<Point>> vHulls;
                            for (vector<Point> vContour : m_vContours)
                            {
                                vector<Point> vHull;
                                convexHull(vContour, vHull);
                                vHulls.emplace_back(vHull);
                            }
                            
                            // Sort contours from biggest to smallest.
                            sort(vHulls.begin(), vHulls.end(), [](const vector<Point>& c1, const vector<Point>& c2) {	return fabs(contourArea(c1, false)) > fabs(contourArea(c2, false)); });
                            
                            // Remove contours whose area doesn't meet the threshold.
                            vector<vector<Point>> vFilteredHulls;
                            for (vector<Point> vHull : vHulls)
                            {
                                if (contourArea(vHull) >= m_dContrastValue)
                                {
                                    vFilteredHulls.emplace_back(vHull);
                                }
                            }

                            if (vFilteredHulls.size() > 2)
                            {
                                // Draw convex hull contours.
                                polylines(m_pFinalImg, vFilteredHulls, true, Scalar(255, 255, 210), 1);

                                // Store the upper and lower extremes of each hull contour.
                                vector<vector<int>> vHullExtremes;
                                for (vector<Point> vHull : vFilteredHulls)
                                {
                                    // Find and store the bounding rect. (Rect type contains x, y, height, width)
                                    auto val = minmax_element(vHull.begin(), vHull.end(), [](Point const& a, Point const& b) { return a.y < b.y; });
                                    vector<int> vPoint;
                                    vPoint.emplace_back(val.first->x);
                                    vPoint.emplace_back(val.first->y);
                                    vPoint.emplace_back(val.second->x);
                                    vPoint.emplace_back(val.second->y);
                                    vHullExtremes.emplace_back(vPoint);
                                }

                                // Now that we have the lines, find the tallest one.
                                int nMinLineLength = 50;
                                vector<int> vTallestLine1 = {0, 0, 0, nMinLineLength};
                                for (vector<int> vLine : vHullExtremes)
                                {
                                    // Compare the y distance of the line to the currently stored biggest one.
                                    if ((vLine[3] - vLine[1]) > (vTallestLine1[3] - vTallestLine1[1]))
                                    {
                                        vTallestLine1.assign(vLine.begin(), vLine.end());
                                    }
                                }

                                // Remove the line we just found.
                                vHullExtremes.erase(remove(vHullExtremes.begin(), vHullExtremes.end(), vTallestLine1));
                                // Find the next tallest line segment.
                                vector<int>vTallestLine2 = {m_nScreenWidth, 0, m_nScreenWidth, nMinLineLength};
                                for (vector<int> vLine : vHullExtremes)
                                {
                                    // Compare the y distance of the line to the currently stored biggest one.
                                    if ((vLine[3] - vLine[1]) > (vTallestLine2[3] - vTallestLine2[1]))
                                    {
                                        vTallestLine2.assign(vLine.begin(), vLine.end());
                                    }
                                }

                                // Find the center line.
                                vector<int> vCenterLine;
                                if (vTallestLine1[0] < vTallestLine2[0])
                                {
                                    vCenterLine = {(vTallestLine1[0] + ((vTallestLine2[0] - vTallestLine1[0]) / 2)), vTallestLine1[1], (vTallestLine1[2] + ((vTallestLine2[2] - vTallestLine1[2]) / 2)), vTallestLine1[3]};
                                }
                                else
                                {
                                    vCenterLine = {(vTallestLine2[0] + ((vTallestLine1[0] - vTallestLine2[0]) / 2)), vTallestLine1[1], (vTallestLine2[2] + ((vTallestLine1[2] - vTallestLine2[2]) / 2)), vTallestLine1[3]};
                                }

                                // Draw the two tallest line segments and the center line.
                                line(m_pFinalImg, Point(vTallestLine2[0], vTallestLine2[1]), Point(vTallestLine2[2], vTallestLine2[3]), Scalar(255, 0, 0), 3, LINE_4, 0);
                                line(m_pFinalImg, Point(vTallestLine1[0], vTallestLine1[1]), Point(vTallestLine1[2], vTallestLine1[3]), Scalar(255, 0, 0), 3, LINE_4, 0);
                                line(m_pFinalImg, Point(vCenterLine[0], vCenterLine[1]), Point(vCenterLine[2], vCenterLine[3]), Scalar(0, 200, 0), 3, LINE_4, 0);

                                // // Store/convert the vHulls contours into a Mat.
                                // Mat mEdgeImg = Mat::zeros(m_pFinalImg.size(), CV_8UC1);
                                // polylines(mEdgeImg, vHulls, true, Scalar(255, 255, 255), 8);
                                // mEdgeImg.copyTo(m_pDilateImg);
                                // // drawContours(mEdgeImg, vHulls, -1, Scalar(255, 255, 255), 1, LINE_4);

                                // // Setup HoughLinesP function variables.
                                // double dRHO = 1;									// Distance resolution in pixels of the hough grid.
                                // double dTheta = PI / 30;							// Angular resolution in radians of the hough grid.
                                // int nThreshold = 30;								// Minimum number of votes.
                                // double dMinLineLength = 50;							// Minimum number of pixels making up a line.
                                // double dMaxLineGap = 50;							// Maximum gap in pixels between connectable line segments.
                                // // Use HoughLinesP algorithm to detect potential line segments.
                                // vector<Vec4i> vLines;
                                // HoughLinesP(mEdgeImg, vLines, dRHO, dTheta, nThreshold, dMinLineLength, dMaxLineGap);

                                // // Draw the detected lines.
                                // for (Vec4i vLine : vLines)
                                // {
                                // 	// Draw line.
                                // 	line(m_pFinalImg, Point(vLine[0], vLine[1]), Point(vLine[2], vLine[3]), Scalar(0, 0, 255), 4, LINE_4, 0);
                                // }
                                
                                // Sort array based on coordinates (leftmost to rightmost) to make sure contours are adjacent.
                                // sort(vBiggestContours.begin(), vBiggestContours.end(), [](const vector<double>& points1, const vector<double>& points2) { return points1[0] < points2[0]; }); 		// Sorts using nCX location.	
                            }
                        }

                        // // Push position of tracked target.
                        // m_nTargetCenterX = nTargetPositionX - (m_nScreenWidth / 2);
                        // m_nTargetCenterY = -(nTargetPositionY - (m_nScreenHeight / 2));

                        // // Draw how many targets are detected on screen.
                        // putText(m_pFinalImg, ("Targets Detected: " + to_string(vBiggestContours.size())), Point(10, m_pFinalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
                        // // Draw target distance and target crosshairs with error line.
                        // putText(m_pFinalImg, ("size:" + to_string(dBiggestContour)), Point(10, m_pFinalImg.rows - 100), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
                        // line(m_pFinalImg, Point(nTargetPositionX, m_nScreenHeight), Point(nTargetPositionX, 0), Scalar(0, 0, 200), 1, LINE_4, 0);
                        // line(m_pFinalImg, Point(0, nTargetPositionY), Point(m_nScreenWidth, nTargetPositionY), Scalar(0, 0, 200), 1, LINE_4, 0);
                        // //line(m_pFinalImg, Point((m_nScreenWidth / 2), (m_nScreenHeight / 2)), Point(nTargetPositionX, nTargetPositionY), Scalar(200, 0, 0), 2, LINE_4, 0);
                    }
                    else
                    {
                        // This section of code is for the future.

                        // This is for tracking the chessboard.
                        // vector<Point2f> vImagePoints;
                        // vImagePoints.emplace_back(Point2f(0.0, 0.0));
                        // int nTargetPositionX = 0; 
                        // int nTargetPositionY = 0;
                        // m_vSolvePNPValues = SolveObjectPose(vImagePoints, ref(m_pFinalImg), ref(m_pFrame), nTargetPositionX, nTargetPositionY);
                    }
                }

                // Put FPS on image.
                m_nFPS = m_pFPS->FramesPerSec();
                putText(m_pFinalImg, ("Camera FPS: " + to_string(pVideoGetter.GetFPS())), Point(420, m_pFinalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
                putText(m_pFinalImg, ("Processor FPS: " + to_string(m_nFPS)), Point(420, m_pFinalImg.rows - 20), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);

                // If tuning mode is enabled, then output contrast or brightness images.
                if (m_bTuningMode)
                {
                    // m_pContrastImg.copyTo(m_pFinalImg);
                    m_pDilateImg.copyTo(m_pFinalImg);
                }

                // Release garbage mats.
                m_pHSVImg.release();
                m_pBlurImg.release();
                m_pFilterImg.release();
            }
        }
        catch (const exception& e)
        {
            //SetIsStopping(true);
            // Print error to console and show that an error has occured on the screen.
            putText(m_pFinalImg, "Image Processing ERROR", Point(280, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.65, Scalar(0, 0, 250), 1);
            cout << "\nWARNING: MAT corrupt or a runtime error has occured! Frame has been dropped." << "\n" << e.what();
        }

        // If the program stops shutdown the thread.
        if (m_bIsStopping)
        {
            break;
        }
    }

    // Clean-up.
    m_bIsStopped = true;
}

/****************************************************************************
        Description:	Turn negative numbers into -1, positive numbers 
                        into 1, and returns 0 when 0.

        Arguments: 		DOUBLE

        Returns: 		INT
****************************************************************************/
int VideoProcess::SignNum(double dVal)
{
    return (double(0) < dVal) - (dVal < double(0));
}

/****************************************************************************
        Description:	Use the detected object points and real world reference
                        points to estimated the 3D pose of the object.

        Arguments: 		INPUT VECTOR, OUTPUT VECTOR

        Returns: 		OUTPUT VECTOR (6 values)
****************************************************************************/
vector<double> VideoProcess::SolveObjectPose(vector<Point2f> m_pImagePoints, Mat &m_pFinalImg, Mat &m_pFrame, int nTargetPositionX, int nTargetPositionY)
{
    // Create instance variables.
    static int nCount = 0;
    Vec3d					vEulerAngles;
    vector<Point2f>			vPositionVector;
    Mat						vMTXR;
    Mat						vMTXQ;
    Mat						vRotationVectors;
    Mat						vRotationMatrix;
    Mat						vTranslationVectors;
    Mat						vTranslationMatrix;
    Mat						vTRNSP;
    Mat 					mNewCameraMatrix;
    Mat						mUndistort;
    Mat						mGray;
    TermCriteria mTermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.001);

    // Create a vector that stores 0s by default. 
    vector<double>	vObjectPosition;
    vObjectPosition.emplace_back(1);
    vObjectPosition.emplace_back(2);
    vObjectPosition.emplace_back(3);
    vObjectPosition.emplace_back(4);
    vObjectPosition.emplace_back(5);
    vObjectPosition.emplace_back(6);

    // This is for tracking the chessboard.
    // vector<Point3f> chessboards;
    // for (double i = 0; i < 9; i++)			//// These were switched around.
    // {
    // 	for (double j = 0; j < 6; j++)		//// These were switched around.
    // 	{
    // 		chessboards.emplace_back(Point3f(i, j, 0.0));
    // 	}
    // }

    // Catch any anomalies from SolvePNP process. (Like bad input data errors.)
    try
    {
        // Refine the corners from image points.
        // vector<Point2f> corners;
        cvtColor(m_pFinalImg, mGray, COLOR_BGR2GRAY);
        // bool bFound = findChessboardCorners(mGray, Size(6, 9), corners);			// These have a possibility of being switched around.
        cornerSubPix(mGray, m_pImagePoints, Size(11, 11), Size(-1, -1), mTermCriteria);
        // drawChessboardCorners(m_pFinalImg, Size(6, 9), corners, bFound);			// These have a possibility of being switched around.

        // Use real world reference points and image points to estimate object pose.
        bool bSuccess = solvePnP(m_pObjectPoints,					// Object reference points in 3D space.			
                                    m_pImagePoints,					// Object points from the 2D camera image.
                                    m_pCameraMatrix,				// Precalibrated camera matrix. (camera specific)
                                    m_pDistanceCoefficients,		// Precalibrated camera config. (camera specific)
                                    vRotationVectors,				// Storage vector for rotation values.
                                    vTranslationVectors,			// Storage vector for translation values.
                                    false,							// Use the provided rvec and tvec values as initial approximations of the rotation and translation vectors, and further optimize them? (useExtrensicGuess)
                                    SOLVEPNP_ITERATIVE				// Method used for the PNP problem.
                                );
        // bool bSuccess = solvePnPRansac(m_pObjectPoints,						// Object reference points in 3D space.			
        // 									m_pImagePoints,						// Object points from the 2D camera image.
        // 									m_pCameraMatrix,				// Precalibrated camera matrix. (camera specific)
        // 									m_pDistanceCoefficients,		// Precalibrated camera config. (camera specific)
        // 									vRotationVectors,				// Storage vector for rotation values.
        // 									vTranslationVectors,			// Storage vector for translation values.
        // 									false,							// Use the provided rvec and tvec values as initial approximations of the rotation and translation vectors, and further optimize them? (useExtrensicGuess)
        //  									100,							// Number of iterations. (adjust for performance?)
        //  									15.0,							// Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
        // 									0.99,							// Confidence value that the algorithm produces a useful result. 
        //  									noArray(),						// Output vector that contains indices of inliers in objectPoints and imagePoints.
        //  									SOLVEPNP_ITERATIVE				// Method used for the PNP problem.
        // 								);

        // If SolvePNP reports a success, then continue with calculations. Else, keep searching. 
        if (bSuccess)
        {
            // Convert the rotation matrix from the solvePNP function to a rotation vector, or vise versa.
            Rodrigues(vRotationVectors, vRotationMatrix);

            // Calculate the camera x, y, z translation.
            transpose(vRotationMatrix, vTRNSP);
            vTranslationMatrix = -vTRNSP * vTranslationVectors;

            // Calculate the pitch, roll, yaw angles of the camera.
            vEulerAngles = RQDecomp3x3(vRotationMatrix, vMTXR, vMTXQ);

            // Store the calculated object values in the vector.
            vObjectPosition.at(0) = vTranslationMatrix.at<double>(0);
            vObjectPosition.at(1) = vTranslationMatrix.at<double>(1);
            vObjectPosition.at(2) = vTranslationMatrix.at<double>(2);
            vObjectPosition.at(3) = vEulerAngles[0];
            vObjectPosition.at(4) = vEulerAngles[1];
            vObjectPosition.at(5) = vEulerAngles[2];

            // Draw axis vectors.
            drawFrameAxes(m_pFinalImg, m_pCameraMatrix, m_pDistanceCoefficients, vRotationVectors, vTranslationVectors, 20.0);

            // Print status onto image.
            putText(m_pFinalImg, "PNP Status: found match!", Point(50, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);
        }
        else
        {
            // If the object is not found, then put 0s in the vector.
            vObjectPosition.emplace_back(0);
            vObjectPosition.emplace_back(0);
            vObjectPosition.emplace_back(0);
            vObjectPosition.emplace_back(0);
            vObjectPosition.emplace_back(0);
            vObjectPosition.emplace_back(0);

            // Print status onto image.
            putText(m_pFinalImg, "PNP Status: searching...", Point(50, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);
        }

        // Reset toggle if the code ran successfully.
        nCount = 0;
    }
    catch (const exception& e)
    {
        // Print status on screen.
        putText(m_pFinalImg, "PNP Status: point data unsolvable...", Point(50, m_pFinalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);

        // Only print the message to the console once per fail.
        if (nCount <= 100)
        {
            // Print message to console.
            cout << "\nMESSAGE: SolvePNP was unable to process the image data. Moving on...\n" << e.what();

            // Add one error count to toggle.
            nCount++;
        }
    }

    // Return useless stuff for now.
    return vObjectPosition;
}

/****************************************************************************
        Description:	Signals the thread to stop.

        Arguments: 		BOOL

        Returns: 		Nothing
****************************************************************************/
void VideoProcess::SetIsStopping(bool bIsStopping)
{
    this->m_bIsStopping = bIsStopping;
}

/****************************************************************************
        Description:	Gets if the thread has stopped.

        Arguments: 		None

        Returns: 		BOOL
****************************************************************************/
bool VideoProcess::GetIsStopped()
{
    return m_bIsStopped;
}

/****************************************************************************
        Description:	Gets the current FPS of the thread.

        Arguments: 		None

        Returns: 		INT
****************************************************************************/
int VideoProcess::GetFPS()
{
    return m_nFPS;
}