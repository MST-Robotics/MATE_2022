/****************************************************************************
			Description:	Implements the VideoShow Class

			Classes:		VideoShow

			Project:		MATE 2022

			Copyright 2021 MST Design Team - Underwater Robotics.
****************************************************************************/
#include "../Headers/VideoShow.h"
///////////////////////////////////////////////////////////////////////////////


/****************************************************************************
        Description:	VideoShow constructor.

        Arguments:		None

        Derived From:	Nothing
****************************************************************************/
VideoShow::VideoShow()
{
    // Create objects.
    m_pFPS									= new FPS();

    // Initialize member variables.
    m_bIsStopping							= false;
    m_bIsStopped							= false;
}

/****************************************************************************
        Description:	VideoShow destructor.

        Arguments:		None

        Derived From:	Nothing
****************************************************************************/
VideoShow::~VideoShow()
{
    // Delete object pointers.
    delete m_pFPS;

    // Set object pointers as nullptrs.
    m_pFPS = nullptr;
}

/****************************************************************************
        Description:	Method that gives the processed frame to CameraServer.

        Arguments: 		MAT&, VECTOR<CVSOURCE>&, SHARED_TIMED_MUTEX&

        Returns: 		Nothing
****************************************************************************/
void VideoShow::ShowFrame(Mat &m_pFrame, vector<CvSource> &m_vCameraSources, shared_timed_mutex &m_pMutex)
{
    // Give other threads some time.
    this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (1)
    {
        // Increment FPS counter.
        m_pFPS->Increment();

        // Check to make sure frame is not corrupt.
        try
        {
            // Slow thread down to save bandwidth.
            this_thread::sleep_for(std::chrono::milliseconds(25));

            // Acquire resource lock for thread.
            shared_lock<shared_timed_mutex> guard(m_pMutex);

            if (!m_pFrame.empty())
            {
                // Output frame to camera stream.
                m_vCameraSources[0].PutFrame(m_pFrame);
            }
            else
            {
                // Print that frame is empty.
                cout << "WARNING: Frame is empty!" << "\n";
            }
        }
        catch (const exception& e)
        {
            //SetIsStopping(true);
            cout << "WARNING: MAT corrupt. Frame has been dropped." << "\n";
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
}

/****************************************************************************
        Description:	Signals the thread to stop.

        Arguments: 		BOOL

        Returns: 		Nothing
****************************************************************************/
void VideoShow::SetIsStopping(bool bIsStopping)
{
    this->m_bIsStopping = bIsStopping;
}

/****************************************************************************
        Description:	Gets if the thread has stopped.

        Arguments: 		Nothing

        Returns: 		BOOL 
****************************************************************************/
bool VideoShow::GetIsStopped()
{
    return m_bIsStopped;
}

/****************************************************************************
        Description:	Gets the current FPS of the thread.

        Arguments: 		Nothing

        Returns: 		INT
****************************************************************************/
int VideoShow::GetFPS()
{
    return m_nFPS;
}
///////////////////////////////////////////////////////////////////////////////