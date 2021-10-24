/****************************************************************************
		Description:	Implements the FPS Class.

		Classes:		FPS

		Project:		MATE 2022

		Copyright 2021 MST Design Team - Underwater Robotics
****************************************************************************/
#include "../Headers/FPS.h"
///////////////////////////////////////////////////////////////////////////////


/****************************************************************************
			Description:	FPS constructor.

			Arguments:		None

			Derived From:	Nothing
****************************************************************************/
FPS::FPS()
{
    // Initialize member variables.
    iterations = 0;
    startTime = time(0);
    tick = 0;
    FPSCount = 0;
}

/****************************************************************************
			Description:	FPS destructor.

			Arguments:		None

			Derived From:	Nothing
****************************************************************************/
FPS::~FPS()
{

}

/****************************************************************************
			Description:	Counts number of interations.

			Arguments: 		None
	
			Returns: 		Nothing
****************************************************************************/
void FPS::Increment()
{
    iterations++;
}

/****************************************************************************
			Description:	Calculate average FPS.

			Arguments: 		None
	
			Returns: 		INT
****************************************************************************/
int FPS::FramesPerSec()
{
    // Create instance variables.
    time_t timeNow = time(0) - startTime;

    // Calculate FPS.
    if (timeNow - tick >= 1)
    {
        tick++;
        FPSCount = iterations;
        iterations = 0;
    }

    // Return FPS value.
    return FPSCount;
}
///////////////////////////////////////////////////////////////////////////////