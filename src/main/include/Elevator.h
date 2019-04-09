/****************************************************************************
	Description:	Defines the CElevator control class.

	Classes:		CElevator

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Elevator_H
#define Elevator_H

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include "CANPosition.h"

using namespace frc;

// Elevator Constants.
const double dElevatorPPR				=  	 1024.0;						// Pulses per Revolution.
const double dElevatorRPI				= 1 / ((1.625 + 0.118) * 3.14);		// 1 / (Diameter of pulley + Rope * PI)
const double dElevatorHomeUpSpeed		=  	   0.15;						// Homing Up Speed.
const double dElevatorHomeDownSpeed		=  	  -0.50;						// Homing Down Speed.
const double dElevatorMaxSpeed			=  	    1.0;						// Max Finding Speed.
const double dElevatorMinSpeed			=  	   -1.0;						// Min Finding Speed.
const double dElevatorMaxPosition		=  	   62.5;						// Max Distance.
const double dElevatorMinPosition		=  	    0.0;						// Min Distance.
const double dElevatorRampRate			=		0.5;						// Closed Loop Ramp Rate.
const double dElevatorTolerance			=		1.7;						// Positional Tolerance.
const double dElevatorProportional		=		0.92;						// Proportional Gain.
const double dElevatorIntegral			=		0.0;						// Integral Gain.
const double dElevatorDerivative		=		0.0;						// Derivative Gain.
const double dElevatorFeedForward		= 	  1.142;						// Feed Forward.
const double dElevatorAccumIZone		=	   15.0;						// Accumulative I Zone.
const double dElevatorFindingTimeout	=	    7.0;						// Max Finding Time.
const double dElevatorHomingTimeout		=		5.0;						// Max Homing Time.
const double dElevatorMotionMagicAccel	=	 1525.0;						// Acceleration for Motion Magic in RPS.
const double dElevatorMotionMagicCruiseRPM	= 525.0;						// Cruise RPM for Motion Magic.
/////////////////////////////////////////////////////////////////////////////


class CElevator
{
public:
	// Prototypes.
	CElevator();
	~CElevator();
	// Elevator Specific methods.
	void Init();
	void Tick();
	void SetSetpoint(double dSetpoint);
	void ManualMove(bool bUp);
	void Stop();
	void SetSpeed(double dMinimum, double dMaximum);
	// Lifting methods.
	void EngageLift(bool bEnabled);
	void ToggleShortLift();
	void EnableShortLift(bool bEnabled);
	void ToggleStabilizer();
	void EnableStabilizer(bool bEnabled);
	void LiftDrive(double dPercent);
	// One line methods.
	void BackOffHome(bool bBackOff)					{	m_pElevator1->BackOffHome(bBackOff);			};
	bool IsReady()									{	return m_pElevator1->IsReady(); 				};
	bool IsHomingComplete()							{	return m_pElevator1->IsHomingComplete();		};
	double GetSetpoint()							{ 	return m_pElevator1->GetSetpoint();				};
	double GetActual()								{	return m_pElevator1->GetActual();				};
	double GetEncoderCount()						{	return m_pElevator1->GetRawEncoderCounts();		};
	void SetTolerance(double dTolerance)			{	m_pElevator1->SetTolerance(dTolerance);			};
	void StartHoming()								{	m_pElevator1->StartHoming();					};
	bool IsFwdLimitSwitchPressed()					{	return m_pElevator1->IsFwdLimitSwitchPressed();	};
	bool IsRevLimitSwitchPressed()					{	return m_pElevator1->IsRevLimitSwitchPressed();	};
	bool IsLiftSensorHit()							{	return !m_pLiftSensor->Get();					};
	bool IsStabilizerSensorHit()					{	return !m_pStabilizerSensor->Get();				};
	// Testing only methods.
	void TestLiftDrive(double dPercent);
	void TestBrake();

private:
	// Private objects.
	CCANPosition* 	m_pElevator1;
	WPI_TalonSRX* 	m_pElevator2;
	WPI_TalonSRX*	m_pLiftDrive;
	Solenoid*	  	m_pElevatorBrake;
	Solenoid*		m_pLiftLock;
	Solenoid*		m_pShortLift;
	DoubleSolenoid*	m_pStabilizer;
	DigitalInput*	m_pStabilizerSensor;
	DigitalInput*	m_pLiftSensor;
	Timer*			m_pTimer;
	double			m_dDelayStartTime;
	bool			m_bStabilizerExt;
};
/////////////////////////////////////////////////////////////////////////////
#endif