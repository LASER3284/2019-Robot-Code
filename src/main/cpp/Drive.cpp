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
CDrive::CDrive(Joystick* pDriveController1, Joystick* pDriveController2)
{
	// Store a copy of the drive controller.
	m_pDriveController1					= pDriveController1;
	m_pDriveController2					= pDriveController2;

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
		// Get the inputs for the joysticks; Left Stick Y Axis, Right Stick X Axis.
		m_dYAxis = m_pDriveController1->GetAxis(frc::Joystick::AxisType::kYAxis);
		m_dXAxis = m_pDriveController2->GetAxis(frc::Joystick::AxisType::kXAxis);

		// Do deadzone checks to make sure we aren't letting the robot drift with the sticks.
		// Check if Y Axis is within the deadzone.
		if (fabs(m_dYAxis) >= dJoystickDeadzone)
		{
			// Copy the sign of the Axis, "exponentiate" the input.
			m_dYAxis = copysign(pow(m_dYAxis, 4), m_dYAxis);
		}
		else
		{
			// Y Axis is inside the deadzone, set it to zero.
			m_dYAxis = 0.000;
		}

		// Check if Y Axis is within the deadzone.
		if (fabs(m_dXAxis) >= dJoystickDeadzone)
		{
			// Valid X Axis value, apply the divisor.
			m_dXAxis = m_dXAxis / (SmartDashboard::GetNumber("X Axis Divisor", 2.000));
		}
		else
		{
			// X Axis is inside the deadzone, set it to zero.
			m_dXAxis = 0.000;
		}

		// Drive the robot.
		ManualDrive(m_dXAxis, m_dYAxis);
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
void CDrive::ManualDrive(double dJoystickX, double dJoystickY)
{
	// We will be squaring the input manually to driver's tastes.
	m_pRobotDrive->ArcadeDrive(dJoystickY, dJoystickX, false);
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