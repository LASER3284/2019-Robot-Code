/******************************************************************************
	Description:	Defines the CBucket control class.

	Classes:		CBucket

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#ifndef Bucket_H
#define Bucket_H
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

using namespace frc;

// Bucket Constants.
const double dBucketRollerIntakeSpeed 	=  -1.00;
const double dBucketRollerHoldSpeed		=  -0.35;
const double dBucketRollerEjectSpeed	=  	1.00;
const double dBucketRollerSensorCurrent	=  	8.00;
const double dBucketEjectDelay			=  	0.20;
const double dBucketRetractDelay		=  	1.00;
const double dBucketCargoEjectDelay		=  	2.00;
/////////////////////////////////////////////////////////////////////////////

class CBucket
{
public:
	// Prototypes.
	CBucket();
	~CBucket();
	// Methods.
	void Init();
	void Tick();
	void SetBucketState(int nState);
	void TestIntakeMotor(double dPercent);
	void TestActuateHatch();
	void TestEjectHatch();
	void TestIntakeHatch();
	void TestSensors();
	int	 GetBucketState()			{	return m_nBucketState;	};
	void StartEject(bool bStart)					{ m_bStartEject = bStart;	};
	// State Machine.
	void BucketStateMachine();
	enum BucketStates
	{
		eBucketAuto,
		eBucketIdle,
		eBucketHolding,
		eBucketHatchIntake1,
		eBucketHatchIntake2,
		eBucketHatchEject1,
		eBucketHatchEject2,
		eBucketHatchEject3,
		eBucketHatchEject4,
		eBucketCargoIntake1,
		eBucketCargoIntake2,
		eBucketCargoEject1,
		eBucketCargoEject2,
		eBucketCargoEject3,
		eBucketCargoEject4,
		eBucketCargoEject5
	};

private:
	// Pointers.
	WPI_TalonSRX*	m_pIntakeRollers;			// Ball intake rollers.
	Solenoid*		m_pHatchIntake;				// Actuate Hatch claw intake.
	Solenoid*		m_pHatchEject;				// Ejects with 2 plungers.
	Solenoid*		m_pHatchActuator;			// Moves Hatch mechanism linearly.
	DigitalInput*	m_pCargoSensor;				// Detects when the bucket intake has cargo.
	DigitalInput*	m_pHatchSensor;				// Detects when the bucket intake has a hatch.
	Timer*			m_pTimer;
	
	// Member Variables.
	double 				m_dDelayStartTime;
	bool				m_bStartEject;
	BucketStates		m_nBucketState;
};
/////////////////////////////////////////////////////////////////////////////
#endif