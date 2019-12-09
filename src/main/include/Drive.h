/****************************************************************************
	Description:	Defines the CDrive control class.

	Classes:		CDrive

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef Drive_h
#define Drive_h

#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>

using namespace rev;
using namespace frc;

// Drive Constants.
const double dDriveLowOpenLoopRampRate	=	 0.550;		// Open Loop Ramp Rate for low gear.
const double dDriveHighOpenLoopRampRate	=	 0.850;		// Open Loop Ramp Rate for high gear.
const double dDriveLowMaxOutput			=	 1.000;		// Max output for low gear.
const double dDriveHighMaxOutput		= 	 0.650;		// Max output for high gear.
const double dJoystickDeadzone			=	 0.115;		// Joystick Axis Deadzone.
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	CDrive control class.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
class CDrive
{
public:
	CDrive(Joystick* pDriveController1, Joystick* pDriveController2);
	~CDrive();

	void	Init();
	void 	Tick();
	void	Stop();
	void	ManualDrive(double dJoystickX, double dJoystickY);
	void 	Shift(bool bLowGear);
	void	SetJoystickControl(bool bJoystickControl);
	void	SetMotorExpiration(double dTimeout);
	void	SetMotorSafety(bool bEnabled);

	bool	IsDriveInHighGear()														{	return !m_pShiftSolenoid->Get();	};
	void	ClearLeftFaults()														{	m_pLeftDriveMotor1->ClearFaults();	};
	void	ClearRightFaults()														{	m_pRightDriveMotor1->ClearFaults();	};

private:
	// Object pointers.
	Joystick*			m_pDriveController1;
	Joystick*			m_pDriveController2;
	DifferentialDrive*	m_pRobotDrive;
	CANSparkMax*		m_pLeftDriveMotor1;
	CANSparkMax*		m_pLeftDriveMotor2;
	CANSparkMax*		m_pLeftDriveMotor3;
	CANSparkMax*		m_pRightDriveMotor1;
	CANSparkMax*		m_pRightDriveMotor2;
	CANSparkMax*		m_pRightDriveMotor3;
	Encoder*			m_pLeftEncoder;
	Encoder*			m_pRightEncoder;
	Solenoid*			m_pShiftSolenoid;

	// Member variables.
	double 				m_dXAxis;
	double 				m_dYAxis;
	bool				m_bJoystickControl;
};
/////////////////////////////////////////////////////////////////////////////
#endif
