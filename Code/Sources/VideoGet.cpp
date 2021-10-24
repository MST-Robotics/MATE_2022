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
    m_pFPS									= new FPS();

    // Initialize Variables.
    m_bIsStopping							= false;
    m_bIsStopped							= false;

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
    delete m_pFPS;

    // Set object pointers as nullptrs.
    m_pFPS				 = nullptr;
}

/****************************************************************************
        Description:	Grabs frames from camera.

        Arguments: 		MAT&, SHARED_TIMED_MUTEX&

        Returns: 		Nothing
****************************************************************************/
void VideoGet::StartCapture(Mat &m_pFrame, bool &bCameraSourceIndex, bool &bDrivingMode, shared_timed_mutex &m_pMutex)
{
    // Continuously grab camera frames.
    while (1)
    {
        // Increment FPS counter.
        m_pFPS->Increment();

        try
        {
            // Acquire resource lock for thread.
            lock_guard<shared_timed_mutex> guard(m_pMutex);		// unique_lock

            // Read frame from video file.
            cap >> m_pFrame;

            // // If the frame is empty, stop the capture.
            // if (m_vCameraSinks.empty())
            // {
            // 	break;
            // }

            // // Grab frame from either camera1 or camera2.
            // static bool bToggle = false;
            // if (bDrivingMode)
            // {
            // 	// Set camera properties.
            // 	m_vCameras[0].SetBrightness(10);
            // 	m_vCameras[0].SetExposureAuto();
            // 	m_vCameras[0].SetWhiteBalanceAuto();
            // 	m_vCameras[1].SetBrightness(10);
            // 	m_vCameras[1].SetExposureAuto();
            // 	m_vCameras[1].SetWhiteBalanceAuto();
            // 	// Set toggle var.
            // 	bToggle = false;
            // }
            // else
            // {
            // 	// Only set properties once.
            // 	if (!bToggle)
            // 	{
            // 		// Set camera properties.
            // 		m_vCameras[0].SetBrightness(0);
            // 		m_vCameras[0].SetExposureManual(20);
            // 		// Set camera properties.
            // 		m_vCameras[1].SetBrightness(10);
            // 		m_vCameras[1].SetExposureAuto();
            // 		m_vCameras[1].SetWhiteBalanceAuto();
            // 		// Set toggle var.
            // 		bToggle = true;
            // 	}
            // }

            // // Get camera frames.
            // if (bCameraSourceIndex)
            // {
            // 	// Get camera frame.
            // 	m_vCameraSinks[1].GrabFrame(m_pFrame);
            // }
            // else
            // {
            // 	// Get camera frame.
            // 	m_vCameraSinks[0].GrabFrame(m_pFrame);
            // }
        }
        catch (const exception& e)
        {
            //SetIsStopping(true);
            cout << "WARNING: Video data empty or camera not present." << "\n" << e.what() << "\n";
        }

        // Calculate FPS.
        m_nFPS = m_pFPS->FramesPerSec();

        // If the program stops shutdown the thread.
        if (m_bIsStopping)
        {
            break;
        }
    }

    // Clean-up.
    m_bIsStopped = true;
    return;
}

/****************************************************************************
        Description:	Signals the thread to stop.

        Arguments: 		BOOL

        Returns: 		Nothing
****************************************************************************/
void VideoGet::SetIsStopping(bool bIsStopping)
{
    this->m_bIsStopping = bIsStopping;
}

/****************************************************************************
        Description:	Gets if the thread has stopped.

        Arguments: 		None

        Returns: 		BOOL
****************************************************************************/
bool VideoGet::GetIsStopped()
{
    return m_bIsStopped;
}

/****************************************************************************
        Description:	Gets the current FPS of the thread.

        Arguments: 		None

        Returns: 		Int
****************************************************************************/
int VideoGet::GetFPS()
{
    return m_nFPS;
}
///////////////////////////////////////////////////////////////////////////////