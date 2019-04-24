/****************************************************************************
	Description:	Defines the CArm control class.

	Classes:		CArm

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Arm_H
#define Arm_H

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include "CANPosition.h"

using namespace frc;

// Arm Constants.
const double dArmPPR				=  	 1024.0;			// Pulses per Revolution.
const double dArmRPD				= 	1.0/360;			// 1 Rotation / 360 Degrees
const double dArmHomeUpSpeed		=  	   0.00;			// Homing Up Speed.
const double dArmHomeDownSpeed		=  	  -0.50;			// Homing Down Speed.
const double dArmMaxSpeed			=  	    1.0;			// Max Finding Speed.
const double dArmMinSpeed			=  	   -1.0;			// Min Finding Speed.
const double dArmMaxPosition		=  	   85.0;			// Max Degrees.
const double dArmMinPosition		=  	    0.0;			// Min Degrees.
const double dArmRampRate			=	  0.225;			// Closed Loop Ramp Rate.
const double dArmTolerance			=		2.5;			// Positional Tolerance.
const double dArmProportional		=		9.7;			// Proportional Gain.
const double dArmIntegral			=	  0.001;			// Integral Gain.
const double dArmDerivative			=		0.0;			// Derivative Gain.
const double dArmFeedForward		=	  3.121;			// Feed Forward.
const double dArmAccumIZone			=	   15.0;			// Accumulative I Zone.
const double dArmFindingTimeout		=	    3.0;			// Max Finding Time.
const double dArmHomingTimeout		=		3.0;			// Max Homing Time.
const double dArmMotionMagicAccel	=	  120.0;			// Acceleration for Motion Magic in RPS.
const double dArmMotionMagicCruiseRPM  =   60.0;			// Cruise RPM for Motion Magic.
/////////////////////////////////////////////////////////////////////////////


class CArm
{
public:
	// Prototypes.
	CArm();
	~CArm();
	// Methods.
	void Init();
	void Tick();
	void SetSetpoint(double dSetpoint);
	void ManualMove(bool bUp);
	void TestBrake();
	void Stop();
	// One line methods.
	bool IsReady()									{	return m_pArmMotor->IsReady(); 					};
	bool IsHomingComplete()							{	return m_pArmMotor->IsHomingComplete();			};
	double GetSetpoint()							{ 	return m_pArmMotor->GetSetpoint();				};
	double GetActual()								{	return m_pArmMotor->GetActual();				};
	double GetEncoderCount()						{	return m_pArmMotor->GetRawEncoderCounts();		};
	void SetTolerance(double dTolerance)			{	m_pArmMotor->SetTolerance(dTolerance);			};
	void StartHoming()								{	m_pArmMotor->StartHoming();						};
	bool IsFwdLimitSwitchPressed()					{	return m_pArmMotor->IsFwdLimitSwitchPressed();	};
	bool IsRevLimitSwitchPressed()					{	return m_pArmMotor->IsRevLimitSwitchPressed();	};
private:
	// Private objects.
	CCANPosition*	m_pArmMotor;
	Solenoid*		m_pArmBrake;
};
/////////////////////////////////////////////////////////////////////////////
#endif