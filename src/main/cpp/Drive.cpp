/****************************************************************************
	Description:	Implements the CDrive control class.

	Classes:		CDrive

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include <frc/WPILib.h>
#include "Drive.h"
#include "IOMap.h"

using namespace rev;
/////////////////////////////////////////////////////////////////////////////


/****************************************************************************
	Description:	CDrive Constructor.

	Arguments:		Joystick* pDriveController

	Derived From:	Nothing
****************************************************************************/
CDrive::CDrive(Joystick* pDriveController)
{
	// Store a copy of the drive controller.
	m_pDriveController					= pDriveController;

	// Create the object pointers.
	m_pLeftDriveMotor1					= new CANSparkMax(nLeftDriveMotor1, CANSparkMax::MotorType::kBrushless);
	m_pLeftDriveMotor2					= new CANSparkMax(nLeftDriveMotor2, CANSparkMax::MotorType::kBrushless);
	m_pLeftDriveMotor3					= new CANSparkMax(nLeftDriveMotor3, CANSparkMax::MotorType::kBrushless);
	m_pRightDriveMotor1					= new CANSparkMax(nRightDriveMotor1, CANSparkMax::MotorType::kBrushless);
	m_pRightDriveMotor2					= new CANSparkMax(nRightDriveMotor2, CANSparkMax::MotorType::kBrushless);
	m_pRightDriveMotor3					= new CANSparkMax(nRightDriveMotor3, CANSparkMax::MotorType::kBrushless);
	m_pLeftEncoder						= new Encoder(nDriveLeftClockA, nDriveLeftClockB, false, CounterBase::k4X);
	m_pRightEncoder						= new Encoder(nDriveRightClockA, nDriveRightClockB, false, CounterBase::k4X);
	m_pShiftSolenoid					= new Solenoid(nPCM1, nDriveShiftSolenoid);
	m_pRobotDrive						= new DifferentialDrive(*m_pLeftDriveMotor1, *m_pRightDriveMotor1);

	// Initialize member variables.
	m_dXAxis							= 0.000;
	m_dYAxis							= 0.000;
	m_bJoystickControl					= false;
}

/****************************************************************************
	Description:	CDrive Destructor.

	Arguments:		None 

	Derived From:	None
****************************************************************************/
CDrive::~CDrive()
{
	// Delete our object pointers.
	delete	m_pLeftDriveMotor1;
	delete	m_pLeftDriveMotor2;
	delete	m_pLeftDriveMotor3;
	delete	m_pRightDriveMotor1;
	delete	m_pRightDriveMotor2;
	delete	m_pRightDriveMotor3;
	delete 	m_pLeftEncoder;
	delete 	m_pRightEncoder;
	delete	m_pShiftSolenoid;
	delete	m_pRobotDrive;

	// Set object pointers as nullptrs.
	m_pLeftDriveMotor1	= nullptr;
	m_pLeftDriveMotor2	= nullptr;
	m_pLeftDriveMotor3	= nullptr;
	m_pRightDriveMotor1	= nullptr;
	m_pRightDriveMotor2	= nullptr;
	m_pRightDriveMotor3	= nullptr;
	m_pLeftEncoder		= nullptr;
	m_pRightEncoder		= nullptr;
	m_pShiftSolenoid	= nullptr;
	m_pRobotDrive		= nullptr;
}

/****************************************************************************
	Description:	Initialize drive parameters.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CDrive::Init()
{
	// Invert the leader outputs.
	m_pLeftDriveMotor1->SetInverted(true);
	m_pRightDriveMotor1->SetInverted(true);
	// Set the second and third motors on each drive to follow the first motor and invert accordingly.
	m_pLeftDriveMotor2->Follow(*m_pLeftDriveMotor1, true);
	m_pLeftDriveMotor3->Follow(*m_pLeftDriveMotor1, false);
	m_pRightDriveMotor2->Follow(*m_pRightDriveMotor1, true); 
	m_pRightDriveMotor3->Follow(*m_pRightDriveMotor1, false);
	// Set the ramp rate.
	m_pLeftDriveMotor1->SetOpenLoopRampRate(dDriveHighOpenLoopRampRate);
	m_pRightDriveMotor1->SetOpenLoopRampRate(dDriveHighOpenLoopRampRate);
	// Set motor neutral mode to brake.
	m_pLeftDriveMotor1->SetIdleMode(CANSparkMax::IdleMode::kBrake);
	m_pLeftDriveMotor2->SetIdleMode(CANSparkMax::IdleMode::kBrake);
	m_pLeftDriveMotor3->SetIdleMode(CANSparkMax::IdleMode::kBrake);
	m_pRightDriveMotor1->SetIdleMode(CANSparkMax::IdleMode::kBrake);
	m_pRightDriveMotor2->SetIdleMode(CANSparkMax::IdleMode::kBrake);
	m_pRightDriveMotor3->SetIdleMode(CANSparkMax::IdleMode::kBrake);
	// Stop the motors.
	m_pLeftDriveMotor1->StopMotor();
	m_pRightDriveMotor1->StopMotor();
	// Clear any sticky faults.
	m_pLeftDriveMotor1->ClearFaults();
	m_pRightDriveMotor1->ClearFaults();
	// Shift to high gear.
	m_pShiftSolenoid->Set(false);
}

/****************************************************************************
	Description:	Tick - main method that does functionality.
					Called each time through robot main loop to update state.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CDrive::Tick()
{
	if (m_bJoystickControl)
	{
		// Quickturn variable.
		bool bQuickturn = false;

		// Get the Left Trigger axis from the joystick, multiply then add 1 to change to a range of 1->3
		double m_dLeftAxis = (m_pDriveController->GetRawAxis(2) * 2.0) + 1.0;

		// Get the Y axis value from the joystick.
		m_dYAxis = m_pDriveController->GetRawAxis(1);
		// Check if the Y Axis value is outside of the deadzone.
		if (fabs(m_dYAxis) >= dJoystickDeadzone)
		{
			// Valid Y Axis value, apply the divisor.
			m_dYAxis = m_pDriveController->GetRawAxis(1) / m_dLeftAxis;
		}
		else
		{
			// Y Axis is inside the deadzone, set it to zero.
			m_dYAxis = 0.000;
		}

		// Get the X axis value from the joystick.
		m_dXAxis = m_pDriveController->GetRawAxis(4);
		// Check if the X Axis value is outside of the deadzone.
		if (fabs(m_dXAxis) >= dJoystickDeadzone)
		{
			// Valid X Axis value, apply the divisor and reverse the axis.
			m_dXAxis = (-1 * (m_dXAxis / (SmartDashboard::GetNumber("X Axis Divisor", 2.000))));
		}
		else
		{
			// X Axis is inside the deadzone, set it to zero.
			m_dXAxis = 0.000;
		}

		// If not moving forward in low gear, square inputs.
		if ((fabs(m_dYAxis) >= dJoystickDeadzone) && IsDriveInHighGear())
		{
			bQuickturn = false;
		}
		else
		{
			bQuickturn = true;
		}

		// Change Ramp Rate and output depending on current gear.
		if (IsDriveInHighGear())
		{
			// Set the ramp rate.
			m_pLeftDriveMotor1->SetOpenLoopRampRate(dDriveHighOpenLoopRampRate);
			m_pRightDriveMotor1->SetOpenLoopRampRate(dDriveHighOpenLoopRampRate);
			// Cap PercentOutput.
			if (fabs(m_dYAxis) > dDriveHighMaxOutput)
			{
				m_dYAxis = m_dYAxis / (1/dDriveHighMaxOutput);
			}
		}
		else
		{
			// Set the ramp rate.
			m_pLeftDriveMotor1->SetOpenLoopRampRate(dDriveLowOpenLoopRampRate);
			m_pRightDriveMotor1->SetOpenLoopRampRate(dDriveLowOpenLoopRampRate);
			// Cap PercentOutput.
			if (fabs(m_dYAxis) > dDriveLowMaxOutput)
			{
				m_dYAxis = m_dYAxis / (1/dDriveLowMaxOutput);
			}
		}
		
		// Drive the robot.
		ManualDrive(m_dXAxis, m_dYAxis, bQuickturn);
	}

	// Put drive information on SmartDashboard for Auto/Teleop.
	SmartDashboard::PutBoolean("High Gear", IsDriveInHighGear());
	SmartDashboard::PutNumber("Left Drive Raw", m_pLeftEncoder->Get());
	SmartDashboard::PutNumber("Right Drive Raw", m_pRightEncoder->Get());
}
/****************************************************************************
	Description:	Method to manually drive robot.

	Arguments: 		double dJoystickX, double dJoystickY, bool bQuickturn

	Returns: 		Nothing
****************************************************************************/
void CDrive::ManualDrive(double dJoystickX, double dJoystickY, bool bArcadeMode)
{
	m_pRobotDrive->ArcadeDrive(dJoystickY, dJoystickX, bArcadeMode);
}
/****************************************************************************
	Description:	Toggles the solenoids to change from low to high gear.

	Arguments: 		bHighGear - True to shift to high gear.

	Returns: 		Nothing
****************************************************************************/
void CDrive::Shift(bool bHighGear)
{
	m_pShiftSolenoid->Set(!bHighGear);
}

/****************************************************************************
	Description:	SetJoystickControl - Sets the desired joystick control.

	Arguments: 		bool bJoystickControl - True if joystick control enabled,
					false otherwise.

	Returns: 		Nothing
****************************************************************************/
void CDrive::SetJoystickControl(bool bJoystickControl)
{
	m_bJoystickControl = bJoystickControl;
}

/****************************************************************************
	Description:	SetMotorExpiration - Sets the motor safety expiration
					timeout.

	Arguments: 		double dTimeout - Expiration timeout

	Returns: 		Nothing
****************************************************************************/
void CDrive::SetMotorExpiration(double dTimeout)
{
	m_pRobotDrive->SetExpiration(dTimeout);
}

/****************************************************************************
	Description:	SetMotorSafety - Sets the motor safety enabled for the
					drive motors.

	Arguments: 		bool bEnabled - True to set MotorSafetyEnabled

	Returns: 		Nothing
****************************************************************************/
void CDrive::SetMotorSafety(bool bEnabled)
{
	m_pRobotDrive->SetSafetyEnabled(bEnabled);
}
///////////////////////////////////////////////////////////////////////////////