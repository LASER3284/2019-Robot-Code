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

///	m_pDistanceSensor 	= new VL53L1X(115200, 4, 11);

	// Create the object pointers.
	m_csFrontCamera				= CameraServer::GetInstance()->StartAutomaticCapture(0);
///	m_csCameraSink				= CameraServer::GetInstance()->GetVideo(m_csFrontCamera);
///	m_csCameraSource			= CameraServer::GetInstance()->PutVideo("Current View", nImageWidth, nImageHeight);
///	m_Image						= cv::Mat();

	// Set up the front camera.
///	m_csFrontCamera.SetResolution(nImageWidth, nImageHeight);
///	m_csFrontCamera.SetFPS(nCameraFPS);
///	m_csFrontCamera.SetExposureAuto();
}

/****************************************************************************
	Description:	CVision Destructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CVision::~CVision()
{
///	delete m_pDistanceSensor;
///	m_pDistanceSensor = nullptr;
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
	Description:	CameraTick - Called in a loop, update camera server.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CVision::CameraTick()
{
	// Grab the current frame to display.
///	m_csCameraSink.GrabFrame(m_Image);
	
	// Send the current frame to the SmartDashboard.
///	m_csCameraSource.PutFrame(m_Image);
}

/****************************************************************************
	Description:	UpdateDistance - Called to update values on
									 SmartDashboard.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CVision::UpdateDistance()
{
	// Generally the reason for the switch case is that updating
	// these in a loop constantly tends to be destructive, to both
	// the serial communication and processing speed of the Rio.
	// The Arduino also doesn't like it much either.
	/*
	static int nCounter = 1;
	switch(nCounter)
	{
		case 1 :
			SmartDashboard::PutNumber("LidarDistanceLeft", m_pDistanceSensor->GetRawDistance(1));
			break;

		case 2 :
			SmartDashboard::PutNumber("LidarDistanceCenter", m_pDistanceSensor->GetRawDistance(2));
			break;

		case 3 :
			SmartDashboard::PutNumber("LidarDistanceRight", m_pDistanceSensor->GetRawDistance(3));
			// Reset state machine
			nCounter = 0;
			break;

		default :
			break;

	}
	nCounter++;
	*/
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
		// Update distance values for TX1.
///		UpdateDistance();

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

/****************************************************************************
	Description:	SwitchCamera -	Called with a camera ID to change to
								   	the selected camera displayed.

	Arguments: 		int nCameraID

	Returns: 		Nothing
****************************************************************************/
void CVision::SwitchCamera(int nCameraID)
{
	// Change selected camera ID.
///	m_nSelectedCamera = nCameraID;
}
/////////////////////////////////////////////////////////////////////////////