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
    FPSCounter									    = new FPS();
    
    // Initialize member variables.
    kernel								    = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    FPSCount								= 0;
    greenBlurRadius						    = 3;
    horizontalAspect						= 4;
    verticalAspect						    = 3;
    cameraFOV							    = 75;
    screenWidth							    = 640;
    screenHeight							= 480;
    focalLength							    = (screenWidth / 2.0) / tan((cameraFOV * PI / 180.0) / 2.0);
    isStopping							    = false;
    isStopped							    = false;

    ////
    // Setup SolvePNP data.
    ////

    // Reference object points.
    objectPoints.emplace_back(Point3f(39.50, 0.0, 0.0));
    objectPoints.emplace_back(Point3f(29.50, -17.0, 0.0));
    objectPoints.emplace_back(Point3f(9.75, -17.0, 0.0));
    objectPoints.emplace_back(Point3f(0.0, 0.0, 0.0));

    // Precalibrated camera matrix values.
    double mtx[3][3] = {{516.5613698781304, 0.0, 320.38297194779585},		//// PSEye Cam
                        {0.0, 515.9356734667019, 231.73585601568368},
                        {0.0, 0.0, 1.0}};
    // double mtx[3][3] = {{659.3851992714341, 0.0, 306.98918779442675},		//// Lifecam
    // 					{0.0, 659.212123568372, 232.07157473243464},
    // 					{0.0, 0.0, 1.0}};
    cameraMatrix = Mat(3, 3, CV_64FC1, mtx).clone();

    // Precalibration distance/distortion values.
    double dist[5] = {-0.0841024904469607, 0.014864043816324026, -0.00013887041018197853, -0.0014661216967276468, 0.5671907234987197};	//// PSEye Cam
    // double dist[5] = {0.1715327237204972, -1.3255106761114646, 7.713495040297368e-07, -0.0035865453000784634, 2.599132082766894};	//// Lifecam
    distanceCoefficients = Mat(1, 5, CV_64FC1, dist).clone();
}

/****************************************************************************
        Description:	VideoProcess destructor.

        Arguments:		None

        Derived From:	Nothing
****************************************************************************/
VideoProcess::~VideoProcess()
{
    // Delete object pointers.
    delete FPSCounter;

    // Set object pointers as nullptrs.
    FPSCounter = nullptr;
}

/****************************************************************************
        Description:	Processes frames with OpenCV.

        Arguments(dear god help us): MAT&, MAT&, INT&, INT&, DOUBLE&, DOUBLE&, BOOL&, BOOL&, BOOL&, BOOL&, VECTOR<INT>, VECTOR<DOUBLE>, VIDEOGET, SHARED_TIMED_MUTEX&, SHARED_TIMED_MUTEX&

        Returns: 		Nothing
****************************************************************************/
void VideoProcess::Process(Mat &frame, Mat &finalImg, int &targetCenterX, int &targetCenterY, double &contrastValue, double &targetAngle, bool &tuningMode, bool &drivingMode, bool &trackingMode, bool &solvePNPEnabled, vector<int> &trackbarValues, vector<double> &solvePNPValues, VideoGet &VideoGetter, shared_timed_mutex &MutexGet, shared_timed_mutex &MutexShow)
{
    // Give other threads enough time to start before processing camera frames.
    this_thread::sleep_for(std::chrono::milliseconds(800));

    while (1)
    {
        // Increment FPS counter.
        FPSCounter->Increment();

        // Make sure frame is not corrupt.
        try
        {
            // Acquire resource lock for read thread. NOTE: This line has been commented out to improve processing speed. VideoGet takes to long with the resources.
            // shared_lock<shared_timed_mutex> guard(MutexGet);

            if (!frame.empty())
            {
                // Convert image from RGB to HSV.
                //cvtColor(frame, HSVImg, COLOR_BGR2HSV);
                // Acquire resource lock for show thread only after frame has been used.
                unique_lock<shared_timed_mutex> guard(MutexShow);
                // Copy frame to a new mat.
                finalImg = frame.clone();
                // Blur the image.
                blur(frame, blurImg, Size(greenBlurRadius, greenBlurRadius));
                // Filter out specific color in image.
                inRange(blurImg, Scalar(trackbarValues[0], trackbarValues[2], trackbarValues[4]), Scalar(trackbarValues[1], trackbarValues[3], trackbarValues[5]), filterImg);
                // Apply blur to image.
                dilate(filterImg, dilateImg, kernel);

                // Find countours of image.
                findContours(dilateImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);		////RETR_TREE //// TRY CHAIN_APPROX_SIMPLE		//// Not sure what this method of detection does, but it worked before: CHAIN_APPROX_TC89_KCOS

                // Driving mode.
                if (!drivingMode)
                {
                    // Tracking mode. (Pipe or Tape)
                    if (trackingMode)
                    {
                        /****************************************************
                        *			Track pipe target
                        *****************************************************/

                        // Draw all contours in white.
                        // drawContours(finalImg, contours, -1, Scalar(255, 255, 210), 1, LINE_4, hierarchy);

                        if (contours.size() >= 2)
                        {
                            // 'Round off' all contours with convexHull.
                            vector<vector<Point>> vHulls;
                            for (vector<Point> vContour : contours)
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
                                if (contourArea(vHull) >= contrastValue)
                                {
                                    vFilteredHulls.emplace_back(vHull);
                                }
                            }

                            if (vFilteredHulls.size() > 2)
                            {
                                // Draw convex hull contours.
                                polylines(finalImg, vFilteredHulls, true, Scalar(255, 255, 210), 1);

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
                                vector<int>vTallestLine2 = {screenWidth, 0, screenWidth, nMinLineLength};
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
                                line(finalImg, Point(vTallestLine2[0], vTallestLine2[1]), Point(vTallestLine2[2], vTallestLine2[3]), Scalar(255, 0, 0), 3, LINE_4, 0);
                                line(finalImg, Point(vTallestLine1[0], vTallestLine1[1]), Point(vTallestLine1[2], vTallestLine1[3]), Scalar(255, 0, 0), 3, LINE_4, 0);
                                line(finalImg, Point(vCenterLine[0], vCenterLine[1]), Point(vCenterLine[2], vCenterLine[3]), Scalar(0, 200, 0), 3, LINE_4, 0);

                                // // Store/convert the vHulls contours into a Mat.
                                // Mat mEdgeImg = Mat::zeros(finalImg.size(), CV_8UC1);
                                // polylines(mEdgeImg, vHulls, true, Scalar(255, 255, 255), 8);
                                // mEdgeImg.copyTo(dilateImg);
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
                                // 	line(finalImg, Point(vLine[0], vLine[1]), Point(vLine[2], vLine[3]), Scalar(0, 0, 255), 4, LINE_4, 0);
                                // }
                                
                                // Sort array based on coordinates (leftmost to rightmost) to make sure contours are adjacent.
                                // sort(vBiggestContours.begin(), vBiggestContours.end(), [](const vector<double>& points1, const vector<double>& points2) { return points1[0] < points2[0]; }); 		// Sorts using nCX location.	
                            }
                        }

                        // // Push position of tracked target.
                        // targetCenterX = targetPositionX - (screenWidth / 2);
                        // targetCenterY = -(targetPositionY - (screenHeight / 2));

                        // // Draw how many targets are detected on screen.
                        // putText(finalImg, ("Targets Detected: " + to_string(vBiggestContours.size())), Point(10, finalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
                        // // Draw target distance and target crosshairs with error line.
                        // putText(finalImg, ("size:" + to_string(dBiggestContour)), Point(10, finalImg.rows - 100), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
                        // line(finalImg, Point(targetPositionX, screenHeight), Point(targetPositionX, 0), Scalar(0, 0, 200), 1, LINE_4, 0);
                        // line(finalImg, Point(0, targetPositionY), Point(screenWidth, targetPositionY), Scalar(0, 0, 200), 1, LINE_4, 0);
                        // //line(finalImg, Point((screenWidth / 2), (screenHeight / 2)), Point(targetPositionX, targetPositionY), Scalar(200, 0, 0), 2, LINE_4, 0);
                    }
                    else
                    {
                        // This section of code is for the future.

                        // This is for tracking the chessboard.
                        // vector<Point2f> vImagePoints;
                        // vImagePoints.emplace_back(Point2f(0.0, 0.0));
                        // int targetPositionX = 0; 
                        // int targetPositionY = 0;
                        // solvePNPValues = SolveObjectPose(vImagePoints, ref(finalImg), ref(frame), targetPositionX, targetPositionY);
                    }
                }

                // Put FPS on image.
                FPSCount = FPSCounter->FramesPerSec();
                putText(finalImg, ("Camera FPS: " + to_string(VideoGetter.GetFPS())), Point(420, finalImg.rows - 40), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);
                putText(finalImg, ("Processor FPS: " + to_string(FPSCount)), Point(420, finalImg.rows - 20), FONT_HERSHEY_DUPLEX, 0.65, Scalar(200, 200, 200), 1);

                // If tuning mode is enabled, then output contrast or brightness images.
                if (tuningMode)
                {
                    // m_pContrastImg.copyTo(finalImg);
                    dilateImg.copyTo(finalImg);
                }

                // Release garbage mats.
                HSVImg.release();
                blurImg.release();
                filterImg.release();
            }
        }
        catch (const exception& e)
        {
            //SetIsStopping(true);
            // Print error to console and show that an error has occured on the screen.
            putText(finalImg, "Image Processing ERROR", Point(280, finalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.65, Scalar(0, 0, 250), 1);
            cout << "\nWARNING: MAT corrupt or a runtime error has occured! Frame has been dropped." << "\n" << e.what();
        }

        // If the program stops shutdown the thread.
        if (isStopping)
        {
            break;
        }
    }

    // Clean-up.
    isStopped = true;
}

/****************************************************************************
        Description:	Turn negative numbers into -1, positive numbers 
                        into 1, and returns 0 when 0.

        Arguments: 		DOUBLE

        Returns: 		INT
****************************************************************************/
int VideoProcess::SignNum(double val)
{
    return (double(0) < val) - (val < double(0));
}

/****************************************************************************
        Description:	Use the detected object points and real world reference
                        points to estimated the 3D pose of the object.

        Arguments: 		INPUT VECTOR, OUTPUT VECTOR

        Returns: 		OUTPUT VECTOR (6 values)
****************************************************************************/
vector<double> VideoProcess::SolveObjectPose(vector<Point2f> imagePoints, Mat &finalImg, Mat &frame, int targetPositionX, int targetPositionY)
{
    // Create instance variables.
    static int count = 0;
    Vec3d					eulerAngles;
    Mat						MTXR;
    Mat						MTXQ;
    Mat						rotationVectors;
    Mat						rotationMatrix;
    Mat						translationVectors;
    Mat						translationMatrix;
    Mat						TRNSP;
    Mat						gray;
    TermCriteria termCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.001);

    // Create a vector that stores 0s by default. 
    vector<double>	objectPosition;
    objectPosition.emplace_back(1);
    objectPosition.emplace_back(2);
    objectPosition.emplace_back(3);
    objectPosition.emplace_back(4);
    objectPosition.emplace_back(5);
    objectPosition.emplace_back(6);

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
        cvtColor(finalImg, gray, COLOR_BGR2GRAY);
        // bool bFound = findChessboardCorners(gray, Size(6, 9), corners);			// These have a possibility of being switched around.
        cornerSubPix(gray, imagePoints, Size(11, 11), Size(-1, -1), termCriteria);
        // drawChessboardCorners(finalImg, Size(6, 9), corners, bFound);			// These have a possibility of being switched around.

        // Use real world reference points and image points to estimate object pose.
        bool success = solvePnP(objectPoints,					// Object reference points in 3D space.			
                                    imagePoints,					// Object points from the 2D camera image.
                                    cameraMatrix,				// Precalibrated camera matrix. (camera specific)
                                    distanceCoefficients,		// Precalibrated camera config. (camera specific)
                                    rotationVectors,				// Storage vector for rotation values.
                                    translationVectors,			// Storage vector for translation values.
                                    false,							// Use the provided rvec and tvec values as initial approximations of the rotation and translation vectors, and further optimize them? (useExtrensicGuess)
                                    SOLVEPNP_ITERATIVE				// Method used for the PNP problem.
                                );
        // bool success = solvePnPRansac(objectPoints,						// Object reference points in 3D space.			
        // 									imagePoints,						// Object points from the 2D camera image.
        // 									cameraMatrix,				// Precalibrated camera matrix. (camera specific)
        // 									distanceCoefficients,		// Precalibrated camera config. (camera specific)
        // 									rotationVectors,				// Storage vector for rotation values.
        // 									translationVectors,			// Storage vector for translation values.
        // 									false,							// Use the provided rvec and tvec values as initial approximations of the rotation and translation vectors, and further optimize them? (useExtrensicGuess)
        //  									100,							// Number of iterations. (adjust for performance?)
        //  									15.0,							// Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
        // 									0.99,							// Confidence value that the algorithm produces a useful result. 
        //  									noArray(),						// Output vector that contains indices of inliers in objectPoints and imagePoints.
        //  									SOLVEPNP_ITERATIVE				// Method used for the PNP problem.
        // 								);

        // If SolvePNP reports a success, then continue with calculations. Else, keep searching. 
        if (success)
        {
            // Convert the rotation matrix from the solvePNP function to a rotation vector, or vise versa.
            Rodrigues(rotationVectors, rotationMatrix);

            // Calculate the camera x, y, z translation.
            transpose(rotationMatrix, TRNSP);
            translationMatrix = -TRNSP * translationVectors;

            // Calculate the pitch, roll, yaw angles of the camera.
            eulerAngles = RQDecomp3x3(rotationMatrix, MTXR, MTXQ);

            // Store the calculated object values in the vector.
            objectPosition.at(0) = translationMatrix.at<double>(0);
            objectPosition.at(1) = translationMatrix.at<double>(1);
            objectPosition.at(2) = translationMatrix.at<double>(2);
            objectPosition.at(3) = eulerAngles[0];
            objectPosition.at(4) = eulerAngles[1];
            objectPosition.at(5) = eulerAngles[2];

            // Draw axis vectors.
            drawFrameAxes(finalImg, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors, 20.0);

            // Print status onto image.
            putText(finalImg, "PNP Status: found match!", Point(50, finalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);
        }
        else
        {
            // If the object is not found, then put 0s in the vector.
            objectPosition.emplace_back(0);
            objectPosition.emplace_back(0);
            objectPosition.emplace_back(0);
            objectPosition.emplace_back(0);
            objectPosition.emplace_back(0);
            objectPosition.emplace_back(0);

            // Print status onto image.
            putText(finalImg, "PNP Status: searching...", Point(50, finalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);
        }

        // Reset toggle if the code ran successfully.
        count = 0;
    }
    catch (const exception& e)
    {
        // Print status on screen.
        putText(finalImg, "PNP Status: point data unsolvable...", Point(50, finalImg.rows - 440), FONT_HERSHEY_DUPLEX, 0.40, Scalar(0, 0, 250), 1);

        // Only print the message to the console once per fail.
        if (count <= 100)
        {
            // Print message to console.
            cout << "\nMESSAGE: SolvePNP was unable to process the image data. Moving on...\n" << e.what();

            // Add one error count to toggle.
            count++;
        }
    }

    // Return useless stuff for now.
    return objectPosition;
}

/****************************************************************************
        Description:	Signals the thread to stop.

        Arguments: 		BOOL

        Returns: 		Nothing
****************************************************************************/
void VideoProcess::SetIsStopping(bool isStopping)
{
    this->isStopping = isStopping;
}

/****************************************************************************
        Description:	Gets if the thread has stopped.

        Arguments: 		None

        Returns: 		BOOL
****************************************************************************/
bool VideoProcess::GetIsStopped()
{
    return isStopped;
}

/****************************************************************************
        Description:	Gets the current FPS of the thread.

        Arguments: 		None

        Returns: 		INT
****************************************************************************/
int VideoProcess::GetFPS()
{
    return FPSCount;
}