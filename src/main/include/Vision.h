/****************************************************************************
	Description:	Defines the DeepSpace Shovel control class.

	Classes:		CVision

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Vision_H
#define Vision_H

#include <frc/WPILib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cameraserver/CameraServer.h>

class CDrive;

using namespace cs;
using namespace cv;
/////////////////////////////////////////////////////////////////////////////

class CVision
{
public:
	CVision(CDrive* pDrive);
	~CVision();
	void Init();
	void DriveTick();

private:
	// Object pointers.
	CDrive*			m_pDrive;
};
/////////////////////////////////////////////////////////////////////////////
#endif