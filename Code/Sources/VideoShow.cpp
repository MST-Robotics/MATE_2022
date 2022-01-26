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
    FPSCounter							= new FPS();

    // Initialize member variables.
    isStopping							= false;
    isStopped							= false;
}

/****************************************************************************
        Description:	VideoShow destructor.

        Arguments:		None

        Derived From:	Nothing
****************************************************************************/
VideoShow::~VideoShow()
{
    // Delete object pointers.
    delete FPSCounter;

    // Set object pointers as nullptrs.
    FPSCounter = nullptr;
}

/****************************************************************************
        Description:	Method that gives the processed frame to CameraServer.

        Arguments: 		MAT&, VECTOR<CVSOURCE>&, SHARED_TIMED_MUTEX&

        Returns: 		Nothing
****************************************************************************/
void VideoShow::ShowFrame(Mat &frame, vector<CvSource> &cameraSources, shared_timed_mutex &Mutex)
{
    // Give other threads some time.
    this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (1)
    {
        // Increment FPS counter.
        FPSCounter->Increment();

        // Check to make sure frame is not corrupt.
        try
        {
            // Slow thread down to save bandwidth.
            this_thread::sleep_for(std::chrono::milliseconds(25));

            // Acquire resource lock for thread.
            shared_lock<shared_timed_mutex> guard(Mutex);

            if (!frame.empty())
            {
                // Output frame to camera stream.
                cameraSources[0].PutFrame(frame);
            }
            else
            {
                // Print that frame is empty.
                cout << "WARNING: Frame is empty!" << endl;
            }
        }
        catch (const exception& e)
        {
            //SetIsStopping(true);
            cout << "WARNING: MAT corrupt. Frame has been dropped." << endl;
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
}

/****************************************************************************
        Description:	Signals the thread to stop.

        Arguments: 		BOOL

        Returns: 		Nothing
****************************************************************************/
void VideoShow::SetIsStopping(bool isStopping)
{
    this->isStopping = isStopping;
}

/****************************************************************************
        Description:	Gets if the thread has stopped.

        Arguments: 		Nothing

        Returns: 		BOOL 
****************************************************************************/
bool VideoShow::GetIsStopped()
{
    return isStopped;
}

/****************************************************************************
        Description:	Gets the current FPS of the thread.

        Arguments: 		Nothing

        Returns: 		INT
****************************************************************************/
int VideoShow::GetFPS()
{
    return FPSCount;
}
///////////////////////////////////////////////////////////////////////////////