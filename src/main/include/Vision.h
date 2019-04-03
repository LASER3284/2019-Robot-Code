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
///#include "VL53L1X.h"

class CDrive;

using namespace cs;
using namespace cv;

// Camera constants.
const int nImageWidth			=  320;
const int nImageHeight			=  240;
const int nCameraFPS				=  	15;
/////////////////////////////////////////////////////////////////////////////

class CVision
{
public:
	CVision(CDrive* pDrive);
	~CVision();
	void Init();
	void UpdateDistance();
	void CameraTick();
	void DriveTick();
	void SwitchCamera(int nCameraID);

private:
	// Object pointers.
	cs::UsbCamera			m_csFrontCamera;		
	cs::CvSink				m_csCameraSink;
	cs::CvSource			m_csCameraSource;
	cv::Mat					m_Image;

	CDrive*			m_pDrive;
///	VL53L1X*		m_pDistanceSensor;
};
/////////////////////////////////////////////////////////////////////////////
#endif