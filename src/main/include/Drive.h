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
const double dDriveOpenLoopRampRate		=	 0.700;		// Open Loop Ramp Rate.
const double dJoystickDeadzone			=	 0.20;		// Joystick Axis Deadzone.
const double dJoystickBandwidth			=	 1.000;		// Filter for ramping joystick values.
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	CDrive control class.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
class CDrive
{
public:
	CDrive(Joystick* pDriveController);
	~CDrive();

	void	Init();
	void 	Tick();
	void	Stop();
	void	ManualDrive(double dJoystickX, double dJoystickY, bool bQuickturn = true);
	void 	Shift(bool bLowGear);
	void	SetJoystickControl(bool bJoystickControl);
	void	SetMotorExpiration(double dTimeout);
	void	SetMotorSafety(bool bEnabled);

	bool	IsDriveInHighGear()														{	return !m_pShiftSolenoid->Get();	};
	void	ClearLeftFaults()														{	m_pLeftDriveMotor1->ClearFaults();	};
	void	ClearRightFaults()														{	m_pRightDriveMotor1->ClearFaults();	};

private:
	// Object pointers.
	Joystick*			m_pDriveController;
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
