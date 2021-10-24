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
    m_nIterations = 0;
    m_pStartTime = time(0);
    m_nTick = 0;
    m_nFPS = 0;
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
    m_nIterations++;
}

/****************************************************************************
			Description:	Calculate average FPS.

			Arguments: 		None
	
			Returns: 		INT
****************************************************************************/
int FPS::FramesPerSec()
{
    // Create instance variables.
    time_t m_pTimeNow = time(0) - m_pStartTime;

    // Calculate FPS.
    if (m_pTimeNow - m_nTick >= 1)
    {
        m_nTick++;
        m_nFPS = m_nIterations;
        m_nIterations = 0;
    }

    // Return FPS value.
    return m_nFPS;
}
///////////////////////////////////////////////////////////////////////////////