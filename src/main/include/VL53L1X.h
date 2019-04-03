/****************************************************************************
	Description:	Defines the VL53L1X Serial Arduino Communications.

	Classes:		VL53L1X

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef VL53L1X_H
#define VL53L1X_H

#include <frc/SerialPort.h>
using namespace frc;
// Sensor Constants.
const double dMillimetersToInches 		=	 25.4;
const double dMillimetersToMeters 		=	100.0;
const int 	 nDefaultBaudRate			=	 9600;
const int	 nDefaultNumOfBytes			=	 	8;
/////////////////////////////////////////////////////////////////////////////

class VL53L1X : public SerialPort
{
public:
	// Prototypes.
	VL53L1X(int nBaudRate = nDefaultBaudRate, int nNumOfSensors = 1, int nNumOfBytes = nDefaultNumOfBytes);
	~VL53L1X();
	// Methods.
	double 	GetRawDistance(int SensorID = 0);
	double	GetInches(int nSensorID = 0);
	double 	GetMeters(int nSensorID = 0);
	void	SetBufferSize(int nNumOfBytes = nDefaultNumOfBytes)		{ m_nNumOfBytes = nNumOfBytes;	};

private:
	int GetDistance(int nSensorID = 1);
	// Pointer for Serial Buffer.
	int m_nNumOfBytes;
	int m_nNumOfSensors;
	SerialPort* m_pDistanceSensor;

};
/////////////////////////////////////////////////////////////////////////////
#endif