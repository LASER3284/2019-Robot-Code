/****************************************************************************
	Description:	Implements the CArm control class.

	Classes:		CArm

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#include <frc/WPILib.h>
#include "Arm.h"
#include "IOMap.h"
#include "CANPosition.h"
/////////////////////////////////////////////////////////////////////////////

/****************************************************************************
	Description:	CArm Constructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CArm::CArm()
{
    // Create objects.
    m_pArmMotor 	= new CCANPosition(nArmMotor);
    m_pArmBrake 	= new Solenoid(nPCM1, nArmBrakeSolenoid);
}

/****************************************************************************
	Description:	CArm Destructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CArm::~CArm()
{
    // Delete objects.
    delete m_pArmMotor;
    delete m_pArmBrake;
    // Set objects to nullptrs.
    m_pArmMotor 	= nullptr; 
    m_pArmBrake  	= nullptr;
}

/****************************************************************************
	Description:	Initialize Arm parameters.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CArm::Init()
{
    // Set the pulses/revolution.
	m_pArmMotor->SetPulsesPerRev(dArmPPR);
    // Set the revolutions/degree.
    m_pArmMotor->SetRevsPerUnit(dArmRPD);
    // Invert the motor output.
    m_pArmMotor->SetMotorInverted(true);
    // Invert the encoder input.
    m_pArmMotor->SetSensorInverted(false);
    // Set the PID terms.
    m_pArmMotor->SetPIDValues(dArmProportional, dArmIntegral, dArmDerivative);
    // Set positional limits.
    m_pArmMotor->SetSoftLimits(dArmMinPosition, dArmMaxPosition);
    // Set tolerance.
    m_pArmMotor->SetTolerance(dArmTolerance);
    // Set maximum and minimum output.
    m_pArmMotor->SetPeakOutputPercent(dArmMaxSpeed, dArmMinSpeed);
    // Set nominal output.
    m_pArmMotor->SetNominalOutputVoltage(0.00, 0.00);
    // Set the Closed Loop Ramp Rate.
    m_pArmMotor->SetClosedLoopRampRate(dArmRampRate);
    // Set neutral mode to Brake.
    m_pArmMotor->SetMotorNeutralMode(NeutralMode::Brake);
    // Set limit switches to normally closed.
    m_pArmMotor->ConfigLimitSwitches(false, false);
    // Set home speeds in Percent.
    m_pArmMotor->SetHomeSpeeds(dArmHomeUpSpeed, dArmHomeDownSpeed);
    // Set homing timeout.
    m_pArmMotor->SetMaxHomingTime(dArmHomingTimeout);
    // Set finding timeout.
    m_pArmMotor->SetMaxFindingTime(dArmFindingTimeout);
    // Stop the motors.
    m_pArmMotor->Stop();
    // Reset encoder position.
    m_pArmMotor->ResetEncoderPosition();
    // Clear sticky faults.
    m_pArmMotor->ClearStickyFaults();
}

/****************************************************************************
	Description:	Tick - main method that does functionality.
					Called each time through robot main loop to update state.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CArm::Tick()
{
	// Call the CANPosition ticks.
	m_pArmMotor->Tick();

	// Get Arm State.
	State nArmState = m_pArmMotor->GetState();

	// Engage/Disengage brake based on state.
	if ((nArmState == eHomingReverse) ||
		(nArmState == eHomingForward) ||
		(nArmState == eManualForward) ||
		(nArmState == eManualReverse) ||
		(nArmState == eFinding))
	{
		// Disengage brake.
		m_pArmBrake->Set(true);
	}
	else
	{
		// Engage brake.
		m_pArmBrake->Set(false);
	}
	// Put information on SmartDashboard for Auto/Teleop.
	SmartDashboard::PutBoolean("Arm IsReady", 	m_pArmMotor->IsReady());
	SmartDashboard::PutNumber("Arm Setpoint", 	m_pArmMotor->GetSetpoint());
	SmartDashboard::PutNumber("Arm Actual", 	m_pArmMotor->GetActual());
    SmartDashboard::PutBoolean("Arm Brake",     !m_pArmBrake->Get());
	SmartDashboard::PutBoolean("Arm Fwd Switch", m_pArmMotor->IsFwdLimitSwitchPressed());
	SmartDashboard::PutBoolean("Arm Rev Switch", m_pArmMotor->IsRevLimitSwitchPressed());
}

/****************************************************************************
	Description:	SetSetpoint - Sets the desired positional setpoint for
					the Arm motor.

	Arguments: 		double dSetpoint - Setpoint in degree for motor.

	Returns: 		Nothing
****************************************************************************/
void CArm::SetSetpoint(double dSetpoint)
{
    // Set the Setpoint, start moving.
    m_pArmMotor->SetSetpoint(dSetpoint);
}

/****************************************************************************
	Description:	ManualMove - Moves the arm up or down.

	Arguments: 		bool bUp - Chooses whether arm goes up or down.

	Returns: 		Nothing
****************************************************************************/
void CArm::ManualMove(bool bUp)
{
    if (m_pArmMotor->GetState() == eIdle)
	{
		// Set state to go Up or Down manually
		m_pArmMotor->SetState(bUp ? eManualReverse : eManualForward);
	}
}

/****************************************************************************
	Description:	Stop - Stops the motor.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CArm::Stop()
{
    // Stop the motors.
    m_pArmMotor->Stop();
}

/****************************************************************************
	Description:	TestBrake - Toggle brake in test mode.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CArm::TestBrake()
{
	// Toggle Brake solenoid.
	m_pArmBrake->Set(!m_pArmBrake->Get());
}
/////////////////////////////////////////////////////////////////////////////