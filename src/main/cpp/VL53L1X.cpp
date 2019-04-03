/****************************************************************************
	Description:	Implement the VL53L1X Serial Arduino Communications.

	Classes:		VL53L1X

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include <VL53L1X.h>
using namespace frc;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	VL53L1X Constructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
VL53L1X::VL53L1X(int nBaudRate, int nNumOfSensors, int nNumOfBytes) : SerialPort(nBaudRate, Port::kMXP)
{
    m_nNumOfBytes 	= nNumOfBytes;
	m_nNumOfSensors = nNumOfSensors;

	// Configure timeout.
	SetTimeout(0.02);

	// Disable Automatic Termination.
	DisableTermination();
}

/****************************************************************************
	Description:	VL53L1X Destructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
VL53L1X::~VL53L1X()
{

}

/****************************************************************************
	Description:	GetDistance - Private function for getting sensors.

	Arguments: 		int nSensorID

	Returns: 		int dRawDistanceFromSensor
****************************************************************************/
int VL53L1X::GetDistance(int nSensorID)
{
	// Check to make sure Sensor ID is valid.
	if((nSensorID > m_nNumOfSensors) || (nSensorID <= 0))
	{
		return 0;
		// Clayton Says: Hey, this is pretty good...
		// Ok Clayton, I appreciate you going through my code
	}

    // Variables to use.
	char *pBuffer = new char;
	std::string strBuffer;
	std::string strSending = "\x02S" + std::to_string(nSensorID) + "\x03";
	int nDistance;

	// Clear buffer.
///	Reset();
	// Send Request to sensor.
///	printf("Writing: %s\n", strSending.c_str());
	Write(strSending);
    // Read from the Serial.
	Read(pBuffer, m_nNumOfBytes);

	// Check to make sure the buffer isn't empty by timeout. If so, return.
	if ((sizeof(pBuffer) == 0) || (sizeof(pBuffer) > (m_nNumOfBytes + 2)))
	{
		return 0;
	}

///	printf("Buffer: %s\n", pBuffer);

    // Store the buffer read from Serial into a string.
	strBuffer = pBuffer;

	// Find last space, the 4 numbers we need are after it.
	strBuffer = strBuffer.substr(strBuffer.find_last_of(' ') + 1, 4);

    // Store the distance as an integer.
///	printf("substring: %s\n", strBuffer.c_str());
	nDistance = std::atoi(strBuffer.c_str());
///	printf("int: %i\nEnd int\n\n", nDistance);

	// Clean up and return.
///	Reset();
	pBuffer = nullptr;

	free(pBuffer);
	return nDistance;
}

/****************************************************************************
	Description:	GetRawDistance - Returns the distance of the selected
									 Sensor. SensorID 0 is all sensors
									 averaged.

	Arguments: 		int nSensorID

	Returns: 		double dRawDistance
****************************************************************************/
double VL53L1X::GetRawDistance(int nSensorID)
{
	// Check to make sure Sensor ID is valid.
	if ((nSensorID > m_nNumOfSensors) || (nSensorID < 0))
	{
		return 0;
	}

	// Double for storing sensor value.
	double dSensorVal = 0;

	// If SensorID is 0, this means the average between all sensors.
	if (nSensorID == 0)
	{
		for (int i = 1; i >= m_nNumOfSensors; i++)
		{
			// Get Current sensor.
			if (dSensorVal == 0)
			{
				// First sensor can't be divided in an average.
				dSensorVal = GetDistance(i);
			}
			else
			{
				// Second (or greater) sensor, add to average.
				dSensorVal = (dSensorVal + GetDistance(i)) / 2;
			}
		}
	}
	else
	{
		// If sensor is specified, get selected sensor.
		dSensorVal = GetDistance(nSensorID);
	}

	// Return Sensor value.
	return dSensorVal;
}

/****************************************************************************
	Description:	GetInches - Returns the distance in inches.

	Arguments: 		int nSensorID

	Returns: 		double dDistanceInInches
****************************************************************************/
double VL53L1X::GetInches(int nSensorID)
{
    // Variable for returning.
    double dReturnVal;
    // Get Distance.
    int nRawDistance = GetRawDistance(nSensorID);
	
	if(nRawDistance == 0)
    {
        // Distance is invalid, return 0.
        dReturnVal = 0;
    }
    else
    {
        // Valid distance, convert value.
        dReturnVal = (double)nRawDistance / dMillimetersToInches;
    }

    // Return value.
    return dReturnVal;
}

/****************************************************************************
	Description:	GetMeters - Returns distance in meters.

	Arguments: 		int nSensorID

	Returns: 		double dDistanceInMeters
****************************************************************************/
double VL53L1X::GetMeters(int nSensorID)
{
    // Variable for returning.
    double dReturnVal;
    // Get Distance.
    int nRawDistance = GetRawDistance();

    if(nRawDistance == 0)
    {
        // Distance is invalid, return 0.
        dReturnVal = 0;
    }
    else
    {
        // Valid distance, convert value.
        dReturnVal = nRawDistance * dMillimetersToMeters;
    }
    // Return value.
    return dReturnVal;
}
/////////////////////////////////////////////////////////////////////////////