/****************************************************************************
			Description:	Implements the VideoGet Class.

			Classes:		VideoGet

			Project:		MATE 2022

			Copyright 2021 MST Design Team - Underwater Robotics.
****************************************************************************/
#include "../Headers/VideoGet.h"
///////////////////////////////////////////////////////////////////////////////


/****************************************************************************
            Description:	VideoGet constructor.

            Arguments:		None

            Derived From:	Nothing
****************************************************************************/
VideoGet::VideoGet()
{
    // Create objects.
    FPSCounter									= new FPS();

    // Initialize Variables.
    isStopping							= false;
    isStopped							= false;

    // Create VideoCapture object for reading video file.
    cap = VideoCapture();
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CV_CAP_PROP_FPS, 30);
    cap.open(0);
}

/****************************************************************************
        Description:	VideoGet destructor.

        Arguments:		None

        Derived From:	Nothing
****************************************************************************/
VideoGet::~VideoGet()
{
    // Delete object pointers.
    delete FPSCounter;

    // Set object pointers as nullptrs.
    FPSCounter				 = nullptr;
}

/****************************************************************************
        Description:	Grabs frames from camera.

        Arguments: 		MAT&, SHARED_TIMED_MUTEX&

        Returns: 		Nothing
****************************************************************************/
void VideoGet::StartCapture(Mat &frame, bool &cameraSourceIndex, bool &drivingMode, shared_timed_mutex &Mutex)
{
    // Continuously grab camera frames.
    while (1)
    {
        // Increment FPS counter.
        FPSCounter->Increment();

        try
        {
            // Acquire resource lock for thread.
            lock_guard<shared_timed_mutex> guard(Mutex);		// unique_lock

            // Read frame from video file.
            cap >> frame;

            // // If the frame is empty, stop the capture.
            // if (cameraSinks.empty())
            // {
            // 	break;
            // }

            // // Grab frame from either camera1 or camera2.
            // static bool bToggle = false;
            // if (drivingMode)
            // {
            // 	// Set camera properties.
            // 	cameras[0].SetBrightness(10);
            // 	cameras[0].SetExposureAuto();
            // 	cameras[0].SetWhiteBalanceAuto();
            // 	cameras[1].SetBrightness(10);
            // 	cameras[1].SetExposureAuto();
            // 	cameras[1].SetWhiteBalanceAuto();
            // 	// Set toggle var.
            // 	bToggle = false;
            // }
            // else
            // {
            // 	// Only set properties once.
            // 	if (!bToggle)
            // 	{
            // 		// Set camera properties.
            // 		cameras[0].SetBrightness(0);
            // 		cameras[0].SetExposureManual(20);
            // 		// Set camera properties.
            // 		cameras[1].SetBrightness(10);
            // 		cameras[1].SetExposureAuto();
            // 		cameras[1].SetWhiteBalanceAuto();
            // 		// Set toggle var.
            // 		bToggle = true;
            // 	}
            // }

            // // Get camera frames.
            // if (cameraSourceIndex)
            // {
            // 	// Get camera frame.
            // 	cameraSinks[1].GrabFrame(frame);
            // }
            // else
            // {
            // 	// Get camera frame.
            // 	cameraSinks[0].GrabFrame(frame);
            // }
        }
        catch (const exception& e)
        {
            //SetIsStopping(true);
            cout << "WARNING: Video data empty or camera not present." << "\n" << e.what() << endl;
        }

        // Calculate FPS.
        FPSCount = FPSCounter->FramesPerSec();

        // If the program stops shutdown the thread.
        if (isStopping)
        {
            break;
        }
    }

    // Clean-up.
    isStopped = true;
    return;
}

/****************************************************************************
        Description:	Signals the thread to stop.

        Arguments: 		BOOL

        Returns: 		Nothing
****************************************************************************/
void VideoGet::SetIsStopping(bool isStopping)
{
    this->isStopping = isStopping;
}

/****************************************************************************
        Description:	Gets if the thread has stopped.

        Arguments: 		None

        Returns: 		BOOL
****************************************************************************/
bool VideoGet::GetIsStopped()
{
    return isStopped;
}

/****************************************************************************
        Description:	Gets the current FPS of the thread.

        Arguments: 		None

        Returns: 		Int
****************************************************************************/
int VideoGet::GetFPS()
{
    return FPSCount;
}
///////////////////////////////////////////////////////////////////////////////