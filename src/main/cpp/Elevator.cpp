/******************************************************************************
	Description:	Implements the CElevator control class.

	Classes:		CElevator

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include <frc/WPILib.h>
#include "Elevator.h"
#include "IOMap.h"
/////////////////////////////////////////////////////////////////////////////

/****************************************************************************
	Description:	CElevator Constructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CElevator::CElevator()
{
    // Create objects.
    m_pElevator1        = new CCANPosition(nElevatorMotor1);
    m_pElevator2        = new WPI_TalonSRX(nElevatorMotor2);
    m_pLiftDrive        = new WPI_TalonSRX(nLiftDriveMotor);
    m_pElevatorBrake    = new Solenoid(nPCM1, nElevatorBrakeSolenoid);
    m_pLiftLock 		= new Solenoid(nPCM1, nLiftLockSolenoid);
    m_pShortLift        = new Solenoid(nPCM1, nLiftShortSolenoid);
	m_pStabilizer		= new DoubleSolenoid(nPCM2, nLiftHighClimbSolenoid1, nLiftHighClimbSolenoid2);
    m_pLiftSensor       = new DigitalInput(nLiftSensor);
    m_pStabilizerSensor = new DigitalInput(nLiftStabilizerSensor);
    m_pTimer            = new Timer();

    m_dDelayStartTime   = 0.0;
    m_bStabilizerExt    = false;
}

/****************************************************************************
	Description:	CElevator Destructor.

	Arguments:		None

	Derived From:	None
****************************************************************************/
CElevator::~CElevator()
{
    // Delete objects.
    delete m_pElevator1;
    delete m_pElevator2;
    delete m_pLiftDrive;
    delete m_pElevatorBrake;
    delete m_pLiftLock;
    delete m_pShortLift;
    delete m_pStabilizer;
    delete m_pTimer;
    delete m_pLiftSensor;
    delete m_pStabilizerSensor;
    // Set objects to null.
    m_pElevator1 		= nullptr;
    m_pElevator2 		= nullptr;
    m_pLiftDrive        = nullptr;
    m_pElevatorBrake	= nullptr;
    m_pLiftLock         = nullptr;
    m_pShortLift        = nullptr;
    m_pStabilizer       = nullptr;
    m_pTimer            = nullptr;
    m_pLiftSensor       = nullptr;
    m_pStabilizerSensor = nullptr;
}

/****************************************************************************
	Description:	Initialize Elevator parameters.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::Init()
{
    // Set the pulses/revolution.
	m_pElevator1->SetPulsesPerRev(dElevatorPPR);
    // Set the revolutions/inch.
    m_pElevator1->SetRevsPerUnit(dElevatorRPI);
    // Invert the motor output.
    m_pElevator1->SetMotorInverted(false);
    m_pElevator2->SetInverted(false);
    // Invert the encoder input.
    m_pElevator1->SetSensorInverted(false);
    // Set the PID terms.
    m_pElevator1->SetPIDValues(dElevatorProportional, dElevatorIntegral, dElevatorDerivative, dElevatorFeedForward);
    // Set positional limits.
    m_pElevator1->SetSoftLimits(dElevatorMinPosition, dElevatorMaxPosition);
    // Set tolerance.
    m_pElevator1->SetTolerance(dElevatorTolerance);
    // Set maximum and minimum output.
    m_pElevator1->SetPeakOutputPercent(dElevatorMaxSpeed, dElevatorMinSpeed);
    // Set nominal output.
    m_pElevator1->SetNominalOutputVoltage(0.00, 0.00);
    // Set the Closed Loop Ramp Rate.
    m_pElevator1->SetClosedLoopRampRate(dElevatorRampRate);
    // Set neutral mode to Brake.
    m_pElevator1->SetMotorNeutralMode(NeutralMode::Brake);
    // Set limit switches to normally closed.
    m_pElevator1->ConfigLimitSwitches(false, false);
    // Set home speeds in Percent.
    m_pElevator1->SetHomeSpeeds(dElevatorHomeUpSpeed, dElevatorHomeDownSpeed);
    // Set homing timeout.
    m_pElevator1->SetMaxHomingTime(dElevatorHomingTimeout);
    // Set finding timeout.
    m_pElevator1->SetMaxFindingTime(dElevatorFindingTimeout);
    // Configure followers.
    m_pElevator2->Follow(*m_pElevator1->GetMotorPointer());
    // Stop the motors.
    m_pElevator1->Stop();
    // Reset encoder position.
    m_pElevator1->ResetEncoderPosition();
	// Configure to use Motion Magic.
	m_pElevator1->UseMotionMagic(true);
	m_pElevator1->SetAcceleration(dElevatorMotionMagicAccel);
	m_pElevator1->SetCruiseRPM(dElevatorMotionMagicCruiseRPM);
    // Clear sticky faults.
    m_pElevator1->ClearStickyFaults();
    m_pLiftDrive->ClearStickyFaults();
    // Start timer.
    m_pTimer->Start();
}

/****************************************************************************
	Description:	Tick - main method that does functionality.
					Called each time through robot main loop to update state.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::Tick()
{
    // Call the CANPosition ticks.
    m_pElevator1->Tick();

	// Get Elevator State.
	State nElevatorState = m_pElevator1->GetState();

	// Engage/Disengage brake based on state.
	if ((nElevatorState == eHomingReverse) ||
		(nElevatorState == eHomingForward) ||
		(nElevatorState == eManualForward) ||
		(nElevatorState == eManualReverse) ||
		(nElevatorState == eFinding))
	{
		// Disengage brake.
		m_pElevatorBrake->Set(true);
	}
	else
	{
		// Engage brake.
		m_pElevatorBrake->Set(false);
	}

	// Put information on SmartDashboard for Auto/Teleop.
	SmartDashboard::PutBoolean("Elevator IsReady", 	m_pElevator1->IsReady());
	SmartDashboard::PutNumber("Elevator Setpoint", 	m_pElevator1->GetSetpoint());
	SmartDashboard::PutNumber("Elevator Actual", 	m_pElevator1->GetActual());
    SmartDashboard::PutBoolean("Elevator Brake",     !m_pElevatorBrake->Get());
	SmartDashboard::PutBoolean("Elevator Fwd Switch", m_pElevator1->IsFwdLimitSwitchPressed());
	SmartDashboard::PutBoolean("Elevator Rev Switch", m_pElevator1->IsRevLimitSwitchPressed());
    SmartDashboard::PutBoolean("Lift Sensor", IsLiftSensorHit());
    SmartDashboard::PutBoolean("Stabilizer Sensor", IsStabilizerSensorHit());
}

/****************************************************************************
	Description:	SetSetpoint - Sets the desired positional setpoint for
					the elevator motor.

	Arguments: 		double dSetpoint - Setpoint in inches for motor.

	Returns: 		Nothing
****************************************************************************/
void CElevator::SetSetpoint(double dSetpoint)
{
    // Set the setpoint, start moving.
    m_pElevator1->SetSetpoint(dSetpoint);
}

/****************************************************************************
	Description:	Drives the lift motors.

	Arguments: 		double dPercent

	Returns: 		Nothing
****************************************************************************/
void CElevator::LiftDrive(double dPercent)
{
    if (fabs(dPercent) >= 0.2)
    {
        m_pLiftDrive->Set(dPercent);
    }
    else
    {
        m_pLiftDrive->Set(0.0);
    }
}

/****************************************************************************
	Description:	ManualMove - Moves the elevator up or down.

	Arguments: 		bool bUp - Chooses whether screw goes up or down.

	Returns: 		Nothing
****************************************************************************/
void CElevator::ManualMove(bool bUp)
{
    if (m_pElevator1->GetState() == eIdle)
	{
		// Set state to go Up or Down manually
		m_pElevator1->SetState(bUp ? eManualForward : eManualReverse);
	}
}

/****************************************************************************
	Description:	Stop - Stops the motors on the elevator.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::Stop()
{
    // Stop the motors.
    m_pElevator1->Stop();
}

/****************************************************************************
	Description:	Stop - Stops the motors on the elevator.

	Arguments: 		double dMinimum, double dMaximum

	Returns: 		Nothing
****************************************************************************/
void CElevator::SetSpeed(double dMinimum, double dMaximum)
{
    // Set Peak Speed.
    m_pElevator1->SetPeakOutputPercent(dMaximum, dMinimum);
}

/****************************************************************************
	Description:	TestLiftDrive - Override variables, drive lift motor.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::TestLiftDrive(double dPercent)
{
    m_pLiftDrive->Set(dPercent);
}

/****************************************************************************
	Description:	TestEngageLift - Toggles lift solenoids.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::EngageLift(bool bEnabled)
{
    m_pLiftLock->Set(bEnabled);
}

/****************************************************************************
	Description:	TestBrake - Toggle brake in test mode.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::TestBrake()
{
	// Toggle Brake solenoid.
	m_pElevatorBrake->Set(!m_pElevatorBrake->Get());
}

/****************************************************************************
	Description:	ToggleShortLift - Toggle Short Lifting cylinder.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::ToggleShortLift()
{
    // Toggle Solenoid.
    m_pShortLift->Set(!m_pShortLift->Get());
}

/****************************************************************************
	Description:	EnableShortLift - Enable Short Lifting cylinder.

	Arguments: 		bool bEnabled

	Returns: 		Nothing
****************************************************************************/
void CElevator::EnableShortLift(bool bEnabled)
{
    // Toggle Solenoid.
    m_pShortLift->Set(bEnabled);
}

/****************************************************************************
	Description:	ToggleStabilizer - Toggle stabilizer solenoid.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CElevator::ToggleStabilizer()
{
    // Invert when called.
    m_bStabilizerExt = !m_bStabilizerExt;

    // Set double solenoid as forward or reverse.
    if (m_bStabilizerExt)
    {
        m_pStabilizer->Set(DoubleSolenoid::Value::kForward);
    }
    else
    {
        m_pStabilizer->Set(DoubleSolenoid::Value::kReverse);
    }
    // Set solenoid.
/// m_pStabilizer->Set(bEnabled);
}

/****************************************************************************
	Description:	EnableStabilzer - Enable or disable stabilizer
                                      solenoid.

	Arguments: 		bool bEnabled

	Returns: 		Nothing
****************************************************************************/
void CElevator::EnableStabilizer(bool bEnabled)
{
    m_bStabilizerExt = bEnabled;
    if (m_bStabilizerExt)
    {
        // Extend the Cylinder.
        m_pStabilizer->Set(DoubleSolenoid::Value::kForward);
    }
    else
    {
        // Retract the Cylinder.
        m_pStabilizer->Set(DoubleSolenoid::Value::kReverse);
    }
}
/////////////////////////////////////////////////////////////////////////////