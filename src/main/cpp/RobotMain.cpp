/******************************************************************************
	Description:	2019 DeepSpace Robot Control Software.

	Classes:		CRobotMain

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#include "RobotMain.h"
#include "Drive.h"
#include "Elevator.h"
#include "Arm.h"
#include "Bucket.h"
#include "Vision.h"
#include "IOMap.h"

using namespace frc;
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
	Description:	CRobotMain Constructor.

	Arguments:		None

	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::CRobotMain() : TimedRobot()
{
    // Create the object pointers.
    m_pDriveController  = new Joystick(0);
	m_pAuxController	= new Joystick(1);
    m_pDrive            = new CDrive(m_pDriveController);
	m_pElevator			= new CElevator();
	m_pArm				= new CArm();
	m_pBucket			= new CBucket();
	m_pVision			= new CVision(m_pDrive);
	m_pTimer			= new Timer();
	m_pPDP				= new PowerDistributionPanel();
	m_pLiveWindow		= LiveWindow::GetInstance();
	m_nTeleopState		= eTeleopFinish;
	m_bDriveControllerPOVUpPressed 		= false;
	m_bDriveControllerPOVDownPressed	= false;
	m_bDriveControllerPOVLeftPressed	= false;
	m_bDriveControllerPOVRightPressed	= false;
	m_bDriveControllerButtonAPressed	= false;
	m_bDriveControllerButtonBPressed	= false;
	m_bDriveControllerButtonXPressed	= false;
	m_bDriveControllerButtonYPressed	= false;
	m_bDriveControllerButtonLSPressed	= false;
	m_bDriveControllerButtonRSPressed	= false;
	m_bAuxControllerButtonAPressed		= false;
	m_bAuxControllerButtonBPressed		= false;
	m_bAuxControllerButtonXPressed		= false;
	m_bAuxControllerButtonYPressed		= false;
	m_bAuxControllerButtonRBPressed		= false;
	m_bAuxControllerButtonLBPressed		= false;
	m_bAuxControllerStopAction			= false;
	m_bAuxControllerButton11Pressed		= false;
	m_bAuxControllerButton12Pressed		= false;
	m_bAuxControllerStartPressed		= false;
	m_bAuxControllerBackPressed			= false;
	m_bAuxControllerStopAction			= false;
	m_bAuxControllerIsArmActive			= m_pAuxController->GetRawButton(9);
}

/******************************************************************************
	Description:	CRobotMain Destructor.

	Arguments:		None

	Derived From:	TimedRobot
******************************************************************************/
CRobotMain::~CRobotMain()
{
    // Delete object pointers.
    delete m_pDriveController;
	delete m_pAuxController;
    delete m_pDrive;
	delete m_pElevator;
	delete m_pArm;
	delete m_pBucket;
	delete m_pTimer;
	delete m_pPDP;
    // Set object pointers to nullptrs.
    m_pDriveController 	= nullptr;
	m_pAuxController	= nullptr;
    m_pDrive 			= nullptr;
	m_pElevator 		= nullptr;
	m_pArm				= nullptr;
	m_pBucket			= nullptr;
	m_pTimer			= nullptr;
	m_pPDP				= nullptr;
}

/******************************************************************************
	RobotInit:	Robot-wide initialization code which is called only once upon
				a power on reset.

	Arguments: 	None

	Returns: 	Nothing
******************************************************************************/
void CRobotMain::RobotInit()
{
	// Call Vision Init.
	m_pVision->Init();
	SmartDashboard::PutBoolean("VisionFreeMode", false);
	// Initialize Drive.
    m_pDrive->Init();
	// Initialize Elevator.
	m_pElevator->Init();
	// Initialize Arm.
	m_pArm->Init();
	// Initialize Bucket.
	m_pBucket->Init();
	// Start timer.
	m_pTimer->Start();
	// Set SmartDashboard variable.
	SmartDashboard::PutNumber("Lift Sensor Timeout", 0.3);
}

/******************************************************************************
	RobotPeriodic:  Called based on the TimedRobot's period any time the program
					is running.

	Arguments: 		None

	Returns: 		Nothing
******************************************************************************/
void CRobotMain::RobotPeriodic()
{
	// Call Vision Camera tick.
	m_pVision->CameraTick();
	m_pVision->DriveTick();
	m_bAuxControllerIsArmActive = m_pAuxController->GetRawButton(9);
	SmartDashboard::PutNumber("Voltage", m_pPDP->GetVoltage());
}

/******************************************************************************
	DisabledInit:	Called one time, each time the robot enters the Disabled
					state.

	Arguments: 		None

	Returns: 		Nothing
******************************************************************************/
void CRobotMain::DisabledInit()
{

}

/******************************************************************************
	DisabledPeriodic:	Called based on the TimedRobot's period any time the
						robot is disabled.

	Arguments: 			None

	Returns: 			Nothing
******************************************************************************/
void CRobotMain::DisabledPeriodic()
{

}

/******************************************************************************
	AutonomousInit:	Called one time, each time the robot enters the autonomous
					state.

	Arguments: 	None

	Returns: 	Nothing
******************************************************************************/
void CRobotMain::AutonomousInit()
{
	TeleopInit();
	// Enable Joystick Control.
	m_pDrive->SetJoystickControl(true);
	// On Init, reset state machine to Idle.
	m_nTeleopState = eAutoStart;
}

/******************************************************************************
	AutonomousPeriodic:	Called based on the TimedRobot's period any time the
						robot is in autonomous mode and enabled.

	Arguments: 			None

	Returns: 			Nothing
******************************************************************************/
void CRobotMain::AutonomousPeriodic()
{
	TeleopPeriodic();
}

/******************************************************************************
	TeleopInit:	Called one time, each time the robot enters the teleop state.

	Arguments: 	None

	Returns: 	Nothing
******************************************************************************/
void CRobotMain::TeleopInit()
{
	// Enable Joystick Control.
	m_pDrive->SetJoystickControl(true);
	// Make sure the Cylinders are retracted (if they are extended for whatever reason)
	m_pElevator->EnableShortLift(false);
	m_pElevator->EnableStabilizer(false);
	// On Init, reset state machine to Idle.
	m_nTeleopState = eTeleopIdle;
	m_nLiftState   = eLiftIdle;
}

/******************************************************************************
	TeleopStateMachine - This is the main state machine used during the
						 Operator Control period.

	Arguments: 	None

	Returns: 	Nothing
******************************************************************************/
void CRobotMain::TeleopStateMachine()
{
	// Teleop State Machine.
	switch (m_nTeleopState)
	{
		case eAutoStart :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Autonomous Starting");
			// Close hatch mechanism.
			m_pBucket->SetBucketState(m_pBucket->eBucketAuto);
			// Shift to low gear.
			m_pDrive->Shift(false);
			break;
		/********************************************************************
			eTeleopIdle - Idle mode, return to low position.
		********************************************************************/
		case eTeleopFinish :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Finishing");
			// Set Positions.
			m_pElevator->SetSetpoint(0.0);
			m_pArm->SetSetpoint(0.0);
			// Move to idle.
			m_nTeleopState = eTeleopIdle;
			break;

		case eTeleopIdle :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Idle");
			break;

		/********************************************************************
			eTeleopCargoPickup - Cargo Intake.
		********************************************************************/
		case eTeleopCargoPickup1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Cargo Pickup");
			// Set Positions.
			m_pElevator->SetSetpoint(0.0);
			m_pArm->SetSetpoint(dArmCargoFloorSetpoint);
			// Start intake.
			m_pBucket->SetBucketState(m_pBucket->eBucketCargoIntake1);
			// Move to next state.
			m_nTeleopState = eTeleopCargoPickup2;
			break;
		
		case eTeleopCargoPickup2 :
			// Check if intake has picked up and secured the ball.
			if ((m_pBucket->GetBucketState() == m_pBucket->eBucketIdle) || m_bAuxControllerStopAction)
			{
				// Go to holding if not already.
				m_pBucket->SetBucketState(m_pBucket->eBucketHolding);
				// Finished, move to idle.
				m_nTeleopState = eTeleopFinish;
			}
			break;

		/********************************************************************
			eTeleopCargoFeeder - Cargo Intake from Feeder station.
		********************************************************************/
		case eTeleopCargoFeeder1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Cargo Feeder");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorCargoFeederSetpoint);
			m_pArm->SetSetpoint(0.0);
			// Start intake.
			m_pBucket->SetBucketState(m_pBucket->eBucketCargoIntake1);
			// Move to next state.
			m_nTeleopState = eTeleopCargoPickup2;
			break;
		
		case eTeleopCargoFeeder2 :
			// Check if intake has picked up and secured the ball.
			if ((m_pBucket->GetBucketState() == m_pBucket->eBucketIdle) || m_bAuxControllerStopAction)
			{
				// Go to holding if not already.
				m_pBucket->SetBucketState(m_pBucket->eBucketHolding);
				// Finished, move to idle.
				m_nTeleopState = eTeleopFinish;
			}
			break;

		/********************************************************************
			eTeleopCargoLowScore - Score cargo in lowest Cargo Port.
		********************************************************************/
		case eTeleopCargoLowScore1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Cargo Low");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorCargoLowSetpoint);
			m_pArm->SetSetpoint(dArmCargoLowSetpoint);
			// Move to next state.
			m_nTeleopState = eTeleopCargoLowScore2;
			break;

		case eTeleopCargoLowScore2 :
			// Check if the elevator and arm has reached setpoint.
			if (m_pElevator->IsReady() || m_pArm->IsReady())
			{
				// Start Eject. Button should be handled internally.
				m_pBucket->SetBucketState(m_pBucket->eBucketCargoEject1);
				// Move to next state.
				m_nTeleopState = eTeleopCargoLowScore3;
			}
			break;

		case eTeleopCargoLowScore3 :
			// Check if the Eject is finished.
			if (m_pBucket->GetBucketState() == m_pBucket->eBucketIdle)
			{
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopCargoMidScore - Score cargo in middle Cargo Port.
		********************************************************************/
		case eTeleopCargoMidScore1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Cargo Mid");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorCargoMidSetpoint);
			m_pArm->SetSetpoint(dArmCargoMidSetpoint);
			// Move to next state.
			m_nTeleopState = eTeleopCargoMidScore2;
			break;

		case eTeleopCargoMidScore2 :
			// Check if the elevator and arm have reached setpoint.
			if (m_pElevator->IsReady() || m_pArm->IsReady())
			{
				// Start Eject.
				m_pBucket->SetBucketState(m_pBucket->eBucketCargoEject1);
				// Move to next state.
				m_nTeleopState = eTeleopCargoMidScore3;
			}
			break;

		case eTeleopCargoMidScore3 :
			// Check if the Eject is finished.
			if (m_pBucket->GetBucketState() == m_pBucket->eBucketIdle)
			{
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopCargoHighScore - Score cargo in top Cargo Port.
		********************************************************************/
		case eTeleopCargoHighScore1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Cargo High");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorCargoHighSetpoint);
			m_pArm->SetSetpoint(dArmCargoHighSetpoint);
			// Move to next state.
			m_nTeleopState = eTeleopCargoHighScore2;
			break;

		case eTeleopCargoHighScore2 :
			// Check if the elevator and arm have reached setpoint.
			if (m_pElevator->IsReady() || m_pArm->IsReady())
			{
				// Start Eject.
				m_pBucket->SetBucketState(m_pBucket->eBucketCargoEject1);
				// Move to next state.
				m_nTeleopState = eTeleopCargoHighScore3;
			}
			break;

		case eTeleopCargoHighScore3 :
			// Check if the Eject is finished.
			if (m_pBucket->GetBucketState() == m_pBucket->eBucketIdle)
			{
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopCargoShip - Score cargo in Cargo Ship.
		********************************************************************/
		case eTeleopCargoShip1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Cargo Ship");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorCargoShipSetpoint);
			m_pArm->SetSetpoint(dArmCargoShipSetpoint);
			// Move to next state.
			m_nTeleopState = eTeleopCargoShip2;
			break;

		case eTeleopCargoShip2 :
			// Check if the elevator and arm have reached setpoint.
			if (m_pElevator->IsReady() || m_pArm->IsReady())
			{
				// Start Eject.
				m_pBucket->SetBucketState(m_pBucket->eBucketCargoEject1);
				// Move to next state.
				m_nTeleopState = eTeleopCargoShip3;
			}
			break;

		case eTeleopCargoShip3 :
			// Check if the Eject is finished.
			if (m_pBucket->GetBucketState() == m_pBucket->eBucketIdle)
			{
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopHatchPickup - Pick up cargo from Driver Station.
		********************************************************************/
		case eTeleopHatchPickup1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Hatch Pickup");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorHatchLowSetpoint);
			m_pArm->SetSetpoint(dArmHatchLowSetpoint);
			// Start intake.
			m_pBucket->SetBucketState(m_pBucket->eBucketHatchIntake1);
			// Move to next state.
			m_nTeleopState = eTeleopHatchPickup2;
			break;

		case eTeleopHatchPickup2 :
			// Check if intake has picked up and secured the hatch.
			if (m_pBucket->GetBucketState() == m_pBucket->eBucketIdle)
			{
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopHatchLowScore - Score hatch on lowest hatch port.
		********************************************************************/
		case eTeleopHatchLowScore1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Hatch Low");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorHatchLowSetpoint);
			m_pArm->SetSetpoint(dArmHatchLowSetpoint);
			// Move to next state.
			m_nTeleopState = eTeleopHatchLowScore2;
			break;

		case eTeleopHatchLowScore2 :
			// Check if the elevator and arm have reached setpoint.
			if (m_pElevator->IsReady() || m_pArm->IsReady())
			{
				// Start Eject.
				m_pBucket->SetBucketState(m_pBucket->eBucketHatchEject1);
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopHatchMidScore - Score hatch on middle hatch port.
		********************************************************************/
		case eTeleopHatchMidScore1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Hatch Mid");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorHatchMidSetpoint);
			m_pArm->SetSetpoint(dArmHatchMidSetpoint);
			// Move to next state.
			m_nTeleopState = eTeleopHatchMidScore2;
			break;

		case eTeleopHatchMidScore2 :
			// Check if the elevator and arm have reached setpoint.
			if (m_pElevator->IsReady() || m_pArm->IsReady())
			{
				// Start Eject.
				m_pBucket->SetBucketState(m_pBucket->eBucketHatchEject1);
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopHatchHighScore - Score hatch on highest hatch port.
		********************************************************************/
		case eTeleopHatchHighScore1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Hatch High");
			// Set Positions.
			m_pElevator->SetSetpoint(dElevatorHatchHighSetpoint);
			m_pArm->SetSetpoint(dArmHatchHighSetpoint);
			// Move to next state.
			m_nTeleopState = eTeleopHatchHighScore2;
			break;

		case eTeleopHatchHighScore2 :
			// Check if the elevator and arm have reached setpoint.
			if (m_pElevator->IsReady() || m_pArm->IsReady())
			{
				// Start Eject.
				m_pBucket->SetBucketState(m_pBucket->eBucketHatchEject1);
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;

		/********************************************************************
			eTeleopLifting - Use lift mechanism for climbing.
		********************************************************************/
		case eTeleopLifting1 :
			// Print out the state.
			SmartDashboard::PutString("Robot State", "Teleop Lifting");
			// Start climbing procedure.
			m_nLiftState = eLift1;
			// Change state.
			m_nTeleopState = eTeleopLifting2;
			break;

		case eTeleopLifting2 :
			// Check if lifting procedure is done.
			if (m_nLiftState == eLiftIdle)
			{
				// Finished, move to idle.
				m_nTeleopState = eTeleopIdle;
			}
			break;
	}
}

/****************************************************************************
	Description:	The State Machine for the Endgame Lift.

	Arguments: 		None

	Returns: 		Nothing
****************************************************************************/
void CRobotMain::LiftStateMachine()
{
	static double dLiftPosition = 0;
	static bool	  bCylinderFired = false;
	switch (m_nLiftState)
	{
		case eLiftIdle :
			// Print out Lift State.
			SmartDashboard::PutString("Lift State", "eLiftIdle");
			break;

        case eLift1 :
            // Print out Lift State.
			SmartDashboard::PutString("Lift State", "Lift starting");
			// Set Elevator speed.
			m_pElevator->SetSpeed(-0.70, 0.40);
            // Move elevator up to starting height.
            m_pElevator->SetSetpoint(dLiftStartHeight);
            // Change state.
            m_nLiftState = eLift2;
            break;

        case eLift2 :
            // Print out Lift State.
			SmartDashboard::PutString("Lift State", "Lift Moving Elevator");
            if (m_pElevator->IsReady())
            {
                // Move arm to setpoint for lifting.
				m_pArm->SetSetpoint(dLiftArmSetpoint);
				// Change state.
           		m_nLiftState = eLift3;
            }
            break;

        case eLift3 :
            // Print out Lift State.
			SmartDashboard::PutString("Lift State", "Lift Moving Arm");
            // Wait for arm to get to position.
            if (m_pArm->IsReady())
            {
                // Flip locks.
                m_pElevator->EngageLift(true);
                // Change state.
                m_nLiftState = eLift4;
            }
            break;

        case eLift4 :
			// Print out Lift State.
			SmartDashboard::PutString("Lift State", "Lift Check Locks");
            // Check if Driver is finished with locking system.
	        if (m_pDriveController->GetRawButton(eStart))
	        {
				// Record Position.
				dLiftPosition = m_pElevator->GetActual();
				// Set to not back off of switch.
				m_pElevator->BackOffHome(false);
				// Start moving elevator.
				m_pElevator->StartHoming();
				// Change state.
				m_nLiftState = eLift5;
	        }
            break;

        case eLift5 :
            // Print out Lift State.
			SmartDashboard::PutString("Lift State", "Lift Stabilizer");
			if (m_pElevator->IsReady())
			{

				// Change state.
				m_nLiftState = eLift6;
			}
            break;

        case eLift6 :
            // Print out Lift State.
            SmartDashboard::PutString("Lift State", "Lift Driving");
			// Start Lift Drive on Right Stick Y.
            m_pElevator->LiftDrive(-m_pDriveController->GetRawAxis(5));
			// As soon as the lift moves, fire cylinder (only once)
			if ((fabs(m_pDriveController->GetRawAxis(5)) >= 0.15) && bCylinderFired == false)
			{
				bCylinderFired = true;
				m_pElevator->EnableStabilizer(true);
				// Change state.
				m_nLiftState = eLift7;
			}
            break;

		case eLift7 :
			// Print out Lift State.
			SmartDashboard::PutString("Lift State", "Lift Checking Sensor");
			// Start Lift Drive on Right Stick Y.
            m_pElevator->LiftDrive(-m_pDriveController->GetRawAxis(5));
			// If lift sensor is hit, change state.
			if (m_pElevator->IsLiftSensorHit())
			{	
				// Get Timer.
				m_dDelayStartTime = m_pTimer->Get();
				// Change state.
				m_nLiftState = eLift8;
			}
			break;

        case eLift8 :
			// Print out Lift State.
            SmartDashboard::PutString("Lift State", "Lift Raising");
			// Wait for arm to be at setpoint, and sensor is still hit after timeout.
			if ((m_pArm->IsReady()) &&
				(m_pTimer->Get() >= (m_dDelayStartTime + SmartDashboard::GetNumber("Lift Sensor Timeout", 0.3))) && 
				(m_pElevator->IsLiftSensorHit()))
			{
				// Stop Lift Drive.
				m_pElevator->LiftDrive(0.0);
				// Move elevator back up.
				m_pElevator->SetSetpoint(dLiftEndHeight);
				// Move to next state.
				m_nLiftState = eLift9;
			}
            break;

        case eLift9 :
			// Print out Lift State.
        	SmartDashboard::PutString("Lift State", "Lift Driving");
			if (m_pElevator->IsReady() && m_pElevator->IsStabilizerSensorHit())
			{
					// Stop drive completely.
					m_pDrive->ManualDrive(0.0, 0.0);
					// Raise cylinder.
					m_pElevator->EnableStabilizer(false);
					// Start timer.
					m_dDelayStartTime = m_pTimer->Get();
					// Move state.
					m_nLiftState = eLift10;
			}
            break;

		case eLift10	:
			// Print out Lift State.
            SmartDashboard::PutString("Lift State", "Lift Ending");
			// Keep drive stopped.
			m_pDrive->ManualDrive(0.0, 0.0);
			// Wait for cylinder to retract.
			if (m_pTimer->Get() >= (m_dDelayStartTime + dLiftFinishDelay))
			{
				// Finished, move to idle.
				m_nLiftState = eLiftIdle;
			}
			break;
	}
}

/******************************************************************************
	TeleopPeriodic:	Called based on the TimedRobot's period any time the
					robot is in teleop mode and enabled.

	Arguments: 		None

	Returns: 		Nothing
******************************************************************************/
void CRobotMain::TeleopPeriodic()
{
	/********************************************************************
		Drive Controller - Shift Gears (LS Toggle)
	********************************************************************/
	// Check to see if the Left Analog Stick was pressed.
	if (m_pDriveController->GetRawButton(eButtonLS) && !m_bDriveControllerButtonLSPressed)
	{
		m_bDriveControllerButtonLSPressed = true;
		// Shift gears.
		m_pDrive->Shift(!m_pDrive->IsDriveInHighGear());
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonLS) && m_bDriveControllerButtonLSPressed)
		{
			m_bDriveControllerButtonLSPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Vision Tracking Mode (RT Hold)
	********************************************************************/
	// Check to see if Right Trigger was pressed
	if (m_pDriveController->GetRawAxis(3) > 0.70)
	{
		SmartDashboard::PutBoolean("VisionIsActive", true);
	}
	else
	{
		SmartDashboard::PutBoolean("VisionIsActive", false);
	}

	/********************************************************************
		Drive Controller - Move to Cargo Pickup position (RB Press)
	********************************************************************/
	// Check to see if Right Bumper was pressed
	if (m_pDriveController->GetRawButton(eButtonRB) && !m_bDriveControllerButtonRBPressed)
	{
		m_bDriveControllerButtonRBPressed = true;
		// Go to Teleop Cargo Pickup.
		m_nTeleopState = eTeleopCargoPickup1;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonRB) && m_bDriveControllerButtonRBPressed)
		{
			m_bDriveControllerButtonRBPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Move to Hatch Pickup position (LB Press)
	********************************************************************/
	// Check to see if Left Bumper was pressed
	if (m_pDriveController->GetRawButton(eButtonLB) && !m_bDriveControllerButtonLBPressed)
	{
		m_bDriveControllerButtonLBPressed = true;
		// Go to Teleop Hatch Pickup.
		m_nTeleopState = eTeleopHatchPickup1;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonLB) && m_bDriveControllerButtonLBPressed)
		{
			m_bDriveControllerButtonLBPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Eject game element. (A Press)
	********************************************************************/
	// Check to see if A Button was pressed
	if (m_pDriveController->GetRawButton(eButtonA) && !m_bDriveControllerButtonAPressed)
	{
		m_bDriveControllerButtonAPressed = true;
		m_pBucket->StartEject(true);
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonA) && m_bDriveControllerButtonAPressed)
		{
			m_bDriveControllerButtonAPressed = false;
			m_pBucket->StartEject(false);
		}
	}

	/********************************************************************
		Drive Controller - Start Lift (Back + Start Press)
	********************************************************************/
	// Check to see if Both Back + Start was pressed.
	if (m_pDriveController->GetRawButton(eBack) && m_pDriveController->GetRawButton(eStart))
	{
		m_nTeleopState = eTeleopLifting1;
	}

	/********************************************************************
		Drive Controller - Manually Eject Cargo (POV Left Press)
	********************************************************************/
	// Check to see if Left POV is pressed.
	if ((m_pDriveController->GetPOV() == 270) && !m_bDriveControllerPOVLeftPressed)
	{
		m_bDriveControllerPOVLeftPressed = true;
		// Manually set the state to eject cargo.
		m_pBucket->SetBucketState(m_pBucket->eBucketCargoEject1);
		m_pBucket->StartEject(true);
	}
	else
	{
		if (m_pDriveController->GetPOV() != 270 && m_bDriveControllerPOVLeftPressed)
		{
			m_bDriveControllerPOVLeftPressed = false;
			m_pBucket->StartEject(false);
		}
	}

	/********************************************************************
		Drive Controller - Manually Eject Hatch (POV Right Press)
	********************************************************************/
	// Check to see if Right POV is pressed.
	if ((m_pDriveController->GetPOV() == 90) && !m_bDriveControllerPOVRightPressed)
	{
		m_bDriveControllerPOVRightPressed = true;
		// Manually set the state to eject hatch.
		m_pBucket->SetBucketState(m_pBucket->eBucketHatchEject1);
		m_pBucket->StartEject(true);
	}
	else
	{
		if (m_pDriveController->GetPOV() != 90 && m_bDriveControllerPOVRightPressed)
		{
			m_bDriveControllerPOVRightPressed = false;
			m_pBucket->StartEject(false);
		}
	}

	/********************************************************************
		Drive Controller - Intake Cargo in Feeder (X Press)
	********************************************************************/
	// Check to see if X Button is pressed.
	if (m_pDriveController->GetRawButton(eButtonX) && !m_bDriveControllerButtonXPressed)
	{
		m_bDriveControllerButtonXPressed = true;
		// Go to Teleop Cargo Feeder.
		m_nTeleopState = eTeleopCargoFeeder1;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonX) && m_bDriveControllerButtonXPressed)
		{
			m_bDriveControllerButtonXPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Eject Cargo in Ship (B Press)
	********************************************************************/
	// Check to see if B Button is pressed.
	if (m_pDriveController->GetRawButton(eButtonB) && !m_bDriveControllerButtonBPressed)
	{
		m_bDriveControllerButtonBPressed = true;
		// Go to Teleop Cargo Ship eject.
		m_nTeleopState = eTeleopCargoShip1;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonB) && m_bDriveControllerButtonBPressed)
		{
			m_bDriveControllerButtonBPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Enable Short Lift. (Y Press)
	********************************************************************/
	// Check to see if Y Button is pressed.
	if (m_pDriveController->GetRawButton(eButtonY) && !m_bDriveControllerButtonYPressed)
	{
		m_bDriveControllerButtonYPressed = true;
		// Toggle Solenoid
		m_pElevator->ToggleShortLift();
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonY) && m_bDriveControllerButtonYPressed)
		{
			m_bDriveControllerButtonYPressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Move to Hatch low position (LB Press)
	********************************************************************/
	// Check to see if Left Bumper was pressed
	if (m_pAuxController->GetRawButton(eButtonLB) && !m_bAuxControllerButtonAPressed)
	{
		m_bAuxControllerButtonLBPressed = true;
		// Go to Teleop Hatch Low position.
		m_nTeleopState = eTeleopHatchLowScore1;
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButtonLB) && m_bAuxControllerButtonLBPressed)
		{
			m_bAuxControllerButtonLBPressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Move to Hatch mid position (X Press)
	********************************************************************/
	// Check to see if X Button was pressed
	if (m_pAuxController->GetRawButton(eButtonX) && !m_bAuxControllerButtonXPressed)
	{
		m_bAuxControllerButtonXPressed = true;
		// Go to Teleop Hatch Mid position.
		m_nTeleopState = eTeleopHatchMidScore1;
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButtonX) && m_bAuxControllerButtonXPressed)
		{
			m_bAuxControllerButtonXPressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Move to Hatch high position (RB Press)
	********************************************************************/
	// Check to see if Right Bumper was pressed
	if (m_pAuxController->GetRawButton(eButtonRB) && !m_bAuxControllerButtonRBPressed)
	{
		m_bAuxControllerButtonRBPressed = true;
		// Go to Teleop Hatch High position.
		m_nTeleopState = eTeleopHatchHighScore1;
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButtonRB) && m_bAuxControllerButtonRBPressed)
		{
			m_bAuxControllerButtonRBPressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Move to Cargo low position (A Press)
	********************************************************************/
	// Check to see if A Button was pressed
	if (m_pAuxController->GetRawButton(eButtonA) && !m_bAuxControllerButtonAPressed)
	{
		m_bAuxControllerButtonAPressed = true;
		// Go to Teleop Cargo Low position.
		m_nTeleopState = eTeleopCargoLowScore1;
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButtonA) && m_bAuxControllerButtonAPressed)
		{
			m_bAuxControllerButtonAPressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Move to Cargo mid position (B Press)
	********************************************************************/
	// Check to see if B Button was pressed
	if (m_pAuxController->GetRawButton(eButtonB) && !m_bAuxControllerButtonBPressed)
	{
		m_bAuxControllerButtonBPressed = true;
		// Go to Teleop Cargo Mid position.
		m_nTeleopState = eTeleopCargoMidScore1;
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButtonB) && m_bAuxControllerButtonBPressed)
		{
			m_bAuxControllerButtonBPressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Move to Cargo high position (Y Press)
	********************************************************************/
	// Check to see if Y Button was pressed
	if (m_pAuxController->GetRawButton(eButtonY) && !m_bAuxControllerButtonYPressed)
	{
		m_bAuxControllerButtonYPressed = true;
		// Go to Teleop Cargo High position.
		m_nTeleopState = eTeleopCargoHighScore1;
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButtonY) && m_bAuxControllerButtonYPressed)
		{
			m_bAuxControllerButtonYPressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Move system up (Start Hold)
	********************************************************************/
	// Check to see if Start was pressed.
	if (m_pAuxController->GetRawButton(eStart) && !m_bAuxControllerStartPressed)
	{
		m_bAuxControllerStartPressed = true;
		if (m_bAuxControllerIsArmActive)
		{
			// Move the arm upwards.
			m_pArm->ManualMove(true);
		}
		else
		{
			// Move the elevator upwards.
			m_pElevator->ManualMove(true);
		}
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eStart) && m_bAuxControllerStartPressed)
		{
			m_bAuxControllerStartPressed = false;
			m_pElevator->Stop();
			m_pArm->Stop();
		}
	}

	/********************************************************************
		Aux Controller - Move system down (Back Hold)
	********************************************************************/
	// Check to see if Back was pressed.
	if (m_pAuxController->GetRawButton(eBack) && !m_bAuxControllerBackPressed)
	{
		m_bAuxControllerBackPressed = true;
		if (m_bAuxControllerIsArmActive)
		{
			// Move the arm upwards.
			m_pArm->ManualMove(false);
		}
		else
		{
			// Move the elevator downwards.
			m_pElevator->ManualMove(false);
		}
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eBack) && m_bAuxControllerBackPressed)
		{
			m_bAuxControllerBackPressed = false;
			m_pElevator->Stop();
			m_pArm->Stop();
		}
	}


	/********************************************************************
		Aux Controller - Home Elevator and Arm (Button 11 Press)
	********************************************************************/
	// Check to see if Button 11 was pressed
	if (m_pAuxController->GetRawButton(eButton11) && !m_bAuxControllerButton11Pressed)
	{
		m_bAuxControllerButton11Pressed = true;
		// Home Elevator and Arm.
		m_pArm->StartHoming();
		m_pElevator->StartHoming();
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButton11) && m_bAuxControllerButton11Pressed)
		{
			m_bAuxControllerButton11Pressed = false;
		}
	}

	/********************************************************************
		Aux Controller - Stop all actions. (Button 12 Press)
	********************************************************************/
	// Check to see if Button 12 was pressed
	if (m_pAuxController->GetRawButton(eButton12) && !m_bAuxControllerButton12Pressed)
	{
		m_bAuxControllerButton12Pressed = true;
		// Cancel all actions.
		m_pArm->Stop();
		m_pElevator->Stop();
		m_pBucket->SetBucketState(m_pBucket->eBucketIdle);
		m_nTeleopState = eTeleopIdle;
	}
	else
	{
		if (!m_pAuxController->GetRawButton(eButton12) && m_bAuxControllerButton12Pressed)
		{
			m_bAuxControllerButton12Pressed = false;
		}
	}

	// Call Drive ticks.
    m_pDrive->Tick();
	// Call Elevator ticks.
	m_pElevator->Tick();
	// Call Arm ticks.
	m_pArm->Tick();
	// Call Bucket ticks,
	m_pBucket->Tick();
	// Call Teleop State Machine.
	TeleopStateMachine();
	LiftStateMachine();
}

/******************************************************************************
	TestInit:	Called one time, each time the robot enters the test state.

	Arguments: 	None

	Returns: 	Nothing
******************************************************************************/
void CRobotMain::TestInit()
{
	// Start LiveWindow instance.
	LiveWindow::GetInstance()->SetEnabled(true);

    // Initialize drive.
	m_pDrive->Init();
	// Initialize arm.
	m_pArm->Init();
	// Initialize elevator.
	m_pElevator->Init();

	// Set the Drive to joystick control.
	m_pDrive->SetJoystickControl(true);

	// MotorSafety improves safety when motors are updated in loops.
	m_pDrive->SetMotorSafety(false);
}

/******************************************************************************
	TestPeriodic:	Called based on the TimedRobot's period any time the
					robot is in test mode and enabled.

	Arguments: 		None

	Returns: 		Nothing
******************************************************************************/
void CRobotMain::TestPeriodic()
{

	/********************************************************************
		Drive Controller - Shift Gears (LS Toggle)
	********************************************************************/
	// Check to see if the Left Analog Button was pressed. (Toggles the drive shift solenoid from low to high gear).
	if (m_pDriveController->GetRawButton(eButtonLS) && !m_bDriveControllerButtonLSPressed)
	{
		m_bDriveControllerButtonLSPressed = true;
		// Shift gears.
		m_pDrive->Shift(!m_pDrive->IsDriveInHighGear());
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonLS) && m_bDriveControllerButtonLSPressed)
		{
			m_bDriveControllerButtonLSPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Move arm up (Start Hold)
	********************************************************************/
	// Check to see if Start was pressed.
	if (m_pDriveController->GetRawButton(eStart) && !m_bDriveControllerStartPressed)
	{
		m_bDriveControllerStartPressed = true;
		// Move the arm upwards.
		m_pArm->ManualMove(true);
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eStart) && m_bDriveControllerStartPressed)
		{
			m_bDriveControllerStartPressed = false;
			// Stop the arm.
			m_pArm->Stop();
		}
	}

	/********************************************************************
		Drive Controller - Move arm down (Back Hold)
	********************************************************************/
	// Check to see if Back was pressed.
	if (m_pDriveController->GetRawButton(eBack) && !m_bDriveControllerBackPressed)
	{
		m_bDriveControllerBackPressed = true;
		// Move the arm downwards.
		m_pArm->ManualMove(false);
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eBack) && m_bDriveControllerBackPressed)
		{
			m_bDriveControllerBackPressed = false;
			// Stop the arm.
			m_pArm->Stop();
		}
	}

	/********************************************************************
		Drive Controller - Move elevator down (LB Hold)
	********************************************************************/
	// Check to see if LB was pressed.
	if (m_pDriveController->GetRawButton(eButtonLB) && !m_bDriveControllerButtonLBPressed)
	{
		m_bDriveControllerButtonLBPressed = true;
		// Move the elevator down.
		m_pElevator->ManualMove(false);
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonLB) && m_bDriveControllerButtonLBPressed)
		{
			m_bDriveControllerButtonLBPressed = false;
			// Stop the elevator.
			m_pElevator->Stop();
		}
	}

	/********************************************************************
		Drive Controller - Move elevator up (RB Hold)
	********************************************************************/
	// Check to see if RB was pressed.
	if (m_pDriveController->GetRawButton(eButtonRB) && !m_bDriveControllerButtonRBPressed)
	{
		m_bDriveControllerButtonRBPressed = true;
		// Move the elevator upwards.
		m_pElevator->ManualMove(true);
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonRB) && m_bDriveControllerButtonRBPressed)
		{
			m_bDriveControllerButtonRBPressed = false;
			// Stop the elevator.
			m_pElevator->Stop();
		}
	}

	/********************************************************************
		Drive Controller - Home Arm (POV Left Button).
	********************************************************************/
	// Check to see if POV Left was pressed
	if ((m_pDriveController->GetPOV() == 270) && !m_bDriveControllerPOVLeftPressed)
	{
		m_pArm->StartHoming();
		m_bDriveControllerPOVLeftPressed = true;
	}
	else
	{
		if ((m_pDriveController->GetPOV() != 270) && m_bDriveControllerPOVLeftPressed)
		{
			m_bDriveControllerPOVLeftPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Home Elevator (POV Right Button).
	********************************************************************/
	// Check to see if POV Right was pressed
	if ((m_pDriveController->GetPOV() == 90) && !m_bDriveControllerPOVRightPressed)
	{
		m_pElevator->StartHoming();
		m_bDriveControllerPOVRightPressed = true;
	}
	else
	{
		if ((m_pDriveController->GetPOV() != 90) && m_bDriveControllerPOVRightPressed)
		{
			m_bDriveControllerPOVRightPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Test Intake Motor (Reverse - Left Trigger)
						  					 (Forward - Right Trigger)
	********************************************************************/
	// Check if the Left Trigger value is outside of the dead zone.
	if ((fabs(m_pDriveController->GetRawAxis(2)) > 0.250) && (fabs(m_pDriveController->GetRawAxis(3)) < 0.250))
	{
		m_pBucket->TestIntakeMotor(m_pDriveController->GetRawAxis(2) * -1.000);
	}
	else
	{
		// Check if the Right Trigger value is outside of the dead zone.
		if ((fabs(m_pDriveController->GetRawAxis(3)) > 0.250) && (fabs(m_pDriveController->GetRawAxis(2)) < 0.250))
		{
			m_pBucket->TestIntakeMotor(m_pDriveController->GetRawAxis(3));
		}
		else
		{
			m_pBucket->TestIntakeMotor(0.000);
		}
	}

	/********************************************************************
		Drive Controller - Drive Lift Motor (Right Y Axis)
	********************************************************************/
	// Check to see Right Y Axis is pressed and within deadzone.
	if (fabs(m_pDriveController->GetRawAxis(5)) > 0.25)
	{
		m_pElevator->TestLiftDrive(m_pDriveController->GetRawAxis(5));
	}
	else
	{
		m_pElevator->TestLiftDrive(0.0);
	}

	/********************************************************************
		Drive Controller - Actuate hatch mechanism. (A Toggle)
	********************************************************************/
	// Check to see if A Button was pressed
	if (m_pDriveController->GetRawButton(eButtonA) && !m_bDriveControllerButtonAPressed)
	{
		m_pBucket->TestActuateHatch();
		m_bDriveControllerButtonAPressed = true;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonA) && m_bDriveControllerButtonAPressed)
		{
			m_bDriveControllerButtonAPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Hatch Eject. (B Toggle)
	********************************************************************/
	// Check to see if B Button was pressed
	if (m_pDriveController->GetRawButton(eButtonB) && !m_bDriveControllerButtonBPressed)
	{
		m_pBucket->TestEjectHatch();
		m_bDriveControllerButtonBPressed = true;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonB) && m_bDriveControllerButtonBPressed)
		{
			m_bDriveControllerButtonBPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Hatch Intake. (X Toggle)
	********************************************************************/
	// Check to see if X Button was pressed
	if (m_pDriveController->GetRawButton(eButtonX) && !m_bDriveControllerButtonXPressed)
	{
		m_pBucket->TestIntakeHatch();
		m_bDriveControllerButtonXPressed = true;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonX) && m_bDriveControllerButtonXPressed)
		{
			m_bDriveControllerButtonXPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Lift Solenoids. (Y Toggle)
	********************************************************************/
	// Check to see if Y Button was pressed
	if (m_pDriveController->GetRawButton(eButtonY) && !m_bDriveControllerButtonYPressed)
	{
		m_pElevator->EngageLift(true);
		m_bDriveControllerButtonYPressed = true;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonY) && m_bDriveControllerButtonYPressed)
		{
			m_pElevator->EngageLift(false);
			m_bDriveControllerButtonYPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Lift Stablizer Solenoid. (RS Toggle)
	********************************************************************/
	// Check to see if Right Stick was pressed
	if (m_pDriveController->GetRawButton(eButtonRS) && !m_bDriveControllerButtonRSPressed)
	{
		m_pElevator->ToggleStabilizer();
		m_bDriveControllerButtonRSPressed = true;
	}
	else
	{
		if (!m_pDriveController->GetRawButton(eButtonRS) && m_bDriveControllerButtonRSPressed)
		{
			m_bDriveControllerButtonRSPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Elevator Brake (POV Down Toggle)
	********************************************************************/
	// Check to see if POV Down was pressed
	if ((m_pDriveController->GetPOV() == 180) && !m_bDriveControllerPOVDownPressed)
	{
		m_pElevator->TestBrake();
		m_bDriveControllerPOVDownPressed = true;
	}
	else
	{
		if ((m_pDriveController->GetPOV() != 180) && m_bDriveControllerPOVDownPressed)
		{
			m_bDriveControllerPOVDownPressed = false;
		}
	}

	/********************************************************************
		Drive Controller - Arm Brake (POV Up Button)
	********************************************************************/
	// Check to see if POV Up was pressed
	if ((m_pDriveController->GetPOV() == 0) && !m_bDriveControllerPOVUpPressed)
	{
		m_pArm->TestBrake();
		m_bDriveControllerPOVUpPressed = true;
	}
	else
	{
		if ((m_pDriveController->GetPOV() != 0) && m_bDriveControllerPOVUpPressed)
		{
			m_bDriveControllerPOVUpPressed = false;
		}
	}

	// Call Drive ticks.
    m_pDrive->Tick();
	// Call Elevator ticks.
	m_pElevator->Tick();
	// Call Arm ticks.
	m_pArm->Tick();
	// Call Bucket sensor loop.
	m_pBucket->TestSensors();
}
///////////////////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<CRobotMain>(); }
#endif
