/******************************************************************************
	Description:	Implements the CBucket control class.

	Classes:		CBucket

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include "Bucket.h"
#include "IOMap.h"
#include <frc/WPILib.h>

using namespace frc;
/////////////////////////////////////////////////////////////////////////////

/****************************************************************************
	Description:	CBucket Constructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CBucket::CBucket()
{
    // Create object pointers.
    m_pIntakeRollers	= new WPI_TalonSRX(nBucketIntakeMotor);
    m_pHatchIntake		= new Solenoid(nPCM1, nBucketIntakeSolenoid);
	m_pHatchEject		= new Solenoid(nPCM1, nBucketEjectSolenoid);
	m_pHatchActuator	= new Solenoid(nPCM1, nBucketLinearSolenoid);
	m_pCargoSensor		= new DigitalInput(nCargoSensor);
	m_pHatchSensor		= new DigitalInput(nHatchSensor);
	m_pTimer			= new Timer();

	m_dDelayStartTime 	= 0.0;
	m_bStartEject		= false;
}

/****************************************************************************
	Description:	CBucket Destructor.

	Arguments:		None

	Derived From:	Nothing
****************************************************************************/
CBucket::~CBucket()
{
	// Delete pointers.
	delete m_pIntakeRollers;
	delete m_pHatchIntake;
	delete m_pHatchEject;
	delete m_pHatchActuator;
	delete m_pCargoSensor;
	delete m_pHatchSensor;
	delete m_pTimer;

	// Set pointers to null.
	m_pIntakeRollers	= nullptr;
	m_pHatchIntake		= nullptr;
	m_pHatchEject		= nullptr;
	m_pHatchActuator	= nullptr;
	m_pCargoSensor		= nullptr;
	m_pHatchSensor		= nullptr;
	m_pTimer			= nullptr;
}

/****************************************************************************
	Description:	Initialize Bucket parameters.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CBucket::Init()
{
	m_pIntakeRollers->SetNeutralMode(NeutralMode::Brake);
	m_nBucketState = eBucketIdle;
	m_pTimer->Start();
}

/****************************************************************************
	Description:	Main method that does functionality.
					Called each time through robot main loop to update state.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CBucket::Tick()
{
	// Call Bucket state machine.
	BucketStateMachine();

	// Put information on SmartDashboard for Auto/Teleop.
	SmartDashboard::PutBoolean("CargoIntakeSensor", m_pCargoSensor->Get());
	SmartDashboard::PutBoolean("HatchIntakeSensor", !m_pHatchSensor->Get());
}

/****************************************************************************
	Description:	The State Machine for the Bucket Class.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CBucket::BucketStateMachine()
{
	switch (m_nBucketState)
	{
		case eBucketAuto :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketIdle");
			// Close hatch intake.
			m_pHatchIntake->Set(true);
			// Extend hatch mech.
			m_pHatchActuator->Set(true);
			break;

		/********************************************************************
			eBucketIdle - Idle, Motors Off.
		********************************************************************/
		case eBucketIdle :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketIdle");
			// Turn Intake off.
			m_pIntakeRollers->Set(0.0);
			break;

		/********************************************************************
			eBucketHolding - Idle, Motors on hold speed.
		********************************************************************/
		case eBucketHolding :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketHolding");
			// Set intake to holding.
			m_pIntakeRollers->Set(dBucketRollerHoldSpeed);
			// Retract Hatch mechanism if not already.
			m_pHatchActuator->Set(false);
			// Retract Eject Plungers if not already.
			m_pHatchEject->Set(false);
			break;

		/********************************************************************
			eBucketHatchIntake - Motors off, Hatch Intake open.
		********************************************************************/
		case eBucketHatchIntake1 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketHatchIntake");
			// Turn Intake off.
			m_pIntakeRollers->Set(0.0);
			// Extend Actuator if not already.
			m_pHatchActuator->Set(true);
			// Retract Eject Plungers.
			m_pHatchEject->Set(false);
			// Open Hatch Intake.
			m_pHatchIntake->Set(false);
			// Move to next state.
			m_nBucketState = eBucketHatchIntake2;
			break;

		case eBucketHatchIntake2 :
			// Check to see if sensor is hit.
			if (!m_pHatchSensor->Get())
			{
				// Sensor tripped, close.
				m_pHatchIntake->Set(true);
				// Move to idle.
				m_nBucketState = eBucketIdle;
			}
			break;

		/********************************************************************
			eBucketHatchEject - Motors off, Hatch Eject with Timer.
		********************************************************************/
		case eBucketHatchEject1 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketHatchIntake1");
			// Turn Intake off.
			m_pIntakeRollers->Set(0.0);
			// Move to next state.
			m_nBucketState = eBucketHatchEject2;
			break;

		case eBucketHatchEject2 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketHatchIntake2");
			// If eject button is pressed, begin eject.
			if (m_bStartEject)
			{
				// Open hatch mech.
				m_pHatchIntake->Set(false);
				// Set timer.
				m_dDelayStartTime = m_pTimer->Get();
				// Move to next state.
				m_nBucketState = eBucketHatchEject3;
			}
			break;

		case eBucketHatchEject3 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketHatchIntake3");
			// If eject delay has been reached, Eject.
			if (m_pTimer->Get() - m_dDelayStartTime >= dBucketEjectDelay)
			{
				// Eject.
				m_pHatchEject->Set(true);
				// Set timer.
				m_dDelayStartTime = m_pTimer->Get();
				// Move to next state.
				m_nBucketState = eBucketHatchEject4;
			}
			break;

		case eBucketHatchEject4 :
			// Print out Bucket State
			SmartDashboard::PutString("Bucket State", "eBucketHatchIntake4");
			// If retract delay has been reached, retract.
			if ((m_pTimer->Get() - m_dDelayStartTime) >= dBucketRetractDelay)
			{
				// Retract.
				m_pHatchEject->Set(false);
				m_pHatchActuator->Set(false);
				// Move to idle.
				m_nBucketState = eBucketIdle;
			}
			break;

		/********************************************************************
			eBucketCargoIntake - Motors on, Hatch mechanism retracted.
		********************************************************************/
		case eBucketCargoIntake1 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketCargoIntake1");
			// Retract hatch mechanisms
			m_pHatchActuator->Set(false);
			m_pHatchIntake->Set(false);
			// Start intake rollers.
			m_pIntakeRollers->Set(dBucketRollerIntakeSpeed);
			// Move to next state.
			m_nBucketState = eBucketCargoIntake2;
			break;
		
		case eBucketCargoIntake2 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketCargoIntake2");
			// Wait until sensor is hit.
			if (m_pCargoSensor->Get())
			{
				// Change intake speed to holding.
				m_pIntakeRollers->Set(dBucketRollerHoldSpeed);
				// Move to holding.
				m_nBucketState = eBucketHolding;
			}
			break;

		/********************************************************************
			eBucketCargoEject - Motors on reverse, Hatch mechanism retracted.
		********************************************************************/
		case eBucketCargoEject1 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketCargoEject1");
			// Retract Hatch mechanisms.
			m_pHatchActuator->Set(false);
			m_pHatchIntake->Set(false);
			// Move to next state.
			m_nBucketState = eBucketCargoEject2;
			break;
			
		case eBucketCargoEject2 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketCargoEject2");
			// Check if the eject button was pressed.
			if (m_bStartEject)
			{
				// Eject the cargo.
				m_pIntakeRollers->Set(dBucketRollerEjectSpeed);
				// Start the Delay timer.
				m_dDelayStartTime = m_pTimer->Get();
				printf("DelayStart: %f", m_dDelayStartTime);
				printf("EjectDelay: %f", dBucketCargoEjectDelay);
				// Move to next state.
				m_nBucketState = eBucketCargoEject3;
			}
			break;

		case eBucketCargoEject3 :
			// Print out Bucket State.
			SmartDashboard::PutString("Bucket State", "eBucketCargoEject1");
			// If stop timeout has been reached, stop motors.
			if ((m_pTimer->Get() - m_dDelayStartTime) >= dBucketCargoEjectDelay)
			{
				printf("Hit after %f Seconds", m_pTimer->Get() - m_dDelayStartTime);
				m_pIntakeRollers->Set(0.0);
				// Move to idle.
				m_nBucketState = eBucketIdle;
			}
			break;
	}
}

/****************************************************************************
	Description:	Sets the Bucket's state machine state.

	Arguments: 		int nState

	Returns: 		Nothing
****************************************************************************/
void CBucket::SetBucketState(int nState)
{
	m_nBucketState = (BucketStates)nState;
}

/****************************************************************************
	Description:	Manually run the Intake Motors.

	Arguments: 		double dPercent

	Returns: 		Nothing
****************************************************************************/
void CBucket::TestIntakeMotor(double dPercent)
{
	m_pIntakeRollers->Set(dPercent);
}

/****************************************************************************
	Description:	Toggle the Hatch Actuator mechanism.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CBucket::TestActuateHatch()
{
	m_pHatchActuator->Set(!m_pHatchActuator->Get());
}

/****************************************************************************
	Description:	Toggle the Hatch Eject.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CBucket::TestEjectHatch()
{
	// Only test eject if Hatch mech is up and not intaking.
	if (m_pHatchActuator->Get() && !m_pHatchIntake->Get())
	{
		m_pHatchEject->Set(!m_pHatchEject->Get()); 
	}
}

/****************************************************************************
	Description:	Toggle the Hatch Intake.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CBucket::TestIntakeHatch()
{
	// Only test eject if Hatch mech is up and not ejecting.
	if (m_pHatchActuator->Get() && !m_pHatchEject->Get())
	{
		m_pHatchIntake->Set(!m_pHatchIntake->Get());
	}
}

/****************************************************************************
	Description:	Loop ran to update sensors in test mode.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CBucket::TestSensors()
{
	SmartDashboard::PutBoolean("CargoIntakeSensor", m_pCargoSensor->Get());
	SmartDashboard::PutBoolean("HatchIntakeSensor", !m_pHatchSensor->Get());
}
/////////////////////////////////////////////////////////////////////////////