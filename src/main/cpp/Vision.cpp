/******************************************************************************
	Description:	Implements the CVision control class.

	Classes:		CVision

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include <frc/WPILIb.h>
#include "Vision.h"
#include "Drive.h"

using namespace cs;
using namespace cv;
using namespace frc;
/////////////////////////////////////////////////////////////////////////////

/****************************************************************************
	Description:	CVision Constructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CVision::CVision(CDrive* pDrive)
{
	// Store the Drive for motor control.
	m_pDrive = pDrive;
}

/****************************************************************************
	Description:	CVision Destructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CVision::~CVision()
{
}

/****************************************************************************
	Description:	Init - Called one time, initializes Vision properties.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CVision::Init()
{
	// TODO: Use init to detect if coprocessor is running properly.
}

/****************************************************************************
	Description:	DriveTick - Called in a loop, calculates values for
								drivetrain and following.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CVision::DriveTick()
{
	double dJoystickX;
	double dJoystickY;
	// If we are in tape tracking mode, Put sensors on Dashboard.
	if (!(SmartDashboard::GetBoolean("VisionFreeMode", false)) && SmartDashboard::GetBoolean("VisionIsActive", false))
	{
		// Drive with virtual values.
		dJoystickX = SmartDashboard::GetNumber("VisionVirtualJoystickX", 0);
		dJoystickY = SmartDashboard::GetNumber("VisionVirtualJoystickY", 0);
		m_pDrive->SetJoystickControl(false);
		m_pDrive->ManualDrive(dJoystickX, dJoystickY);
	}
	else
	{
		// Drive normally.
		m_pDrive->SetJoystickControl(true);
	}
}
/////////////////////////////////////////////////////////////////////////////