/******************************************************************************
	Description:	2019 DeepSpace Robot Control Software.

	Classes:		CRobotMain

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
******************************************************************************/
#ifndef RobotMain_H
#define RobotMain_H

#include <frc/WPILib.h>
#include <frc/LiveWindow/LiveWindow.h>

class CDrive;
class CElevator;
class CArm;
class CBucket;
class CVision;
using namespace frc;

// Teleop Constants.
const double dElevatorCargoLowSetpoint 	= 2.0;
const double dElevatorCargoMidSetpoint	= 28.0;
const double dElevatorCargoHighSetpoint = 54.8;
const double dElevatorCargoShipSetpoint = 17.4;
const double dElevatorCargoFeederSetpoint	= 17.0;
const double dArmCargoFloorSetpoint		= 84.0;
const double dArmCargoLowSetpoint		= 0.00;
const double dArmCargoMidSetpoint		= 0.00;
const double dArmCargoHighSetpoint		= 0.00;
const double dArmCargoShipSetpoint		= 0.00;
const double dElevatorHatchLowSetpoint	= 0.00;
const double dElevatorHatchMidSetpoint	= 8.54;
const double dElevatorHatchHighSetpoint = 37.39;
const double dArmHatchLowSetpoint		= 84.0;
const double dArmHatchMidSetpoint		= 0.00;
const double dArmHatchHighSetpoint		= 0.00;
const double dLiftArmSetpoint			= 86.0;
const double dLiftStartHeight			= 20.5;
const double dLiftEndHeight				= 24.5;
const double dLiftFinishDelay			= 5.00;
const double dLiftStabilizerEngage		= 10.0;
const double dLiftLidarDistance			= 400.0;
////////////////////////////////////////////////////////////////////////////////

class CRobotMain : public TimedRobot
{
public:
	CRobotMain();
	~CRobotMain();

private:
	void RobotInit();
	void RobotPeriodic();

	void DisabledInit();
	void DisabledPeriodic();

	void AutonomousInit();
	void AutonomousPeriodic();

	void TeleopInit();
	void TeleopPeriodic();
	
	void TeleopStateMachine();
	void LiftStateMachine();

	void TestInit();
	void TestPeriodic();

	enum TeleopStates
	{
		eAutoStart,
		eTeleopFinish,
		eTeleopIdle,
		eTeleopHatchPickup1,
		eTeleopHatchPickup2,
		eTeleopHatchLowScore1,
		eTeleopHatchLowScore2,
		eTeleopHatchMidScore1,
		eTeleopHatchMidScore2,
		eTeleopHatchHighScore1,
		eTeleopHatchHighScore2,
		eTeleopCargoPickup1,
		eTeleopCargoPickup2,
		eTeleopCargoFeeder1,
		eTeleopCargoFeeder2,
		eTeleopCargoLowScore1,
		eTeleopCargoLowScore2,
		eTeleopCargoLowScore3,
		eTeleopCargoMidScore1,
		eTeleopCargoMidScore2,
		eTeleopCargoMidScore3,
		eTeleopCargoHighScore1,
		eTeleopCargoHighScore2,
		eTeleopCargoHighScore3,
		eTeleopCargoShip1,
		eTeleopCargoShip2,
		eTeleopCargoShip3,
		eTeleopLifting1,
		eTeleopLifting2
	};

	enum LiftStates
	{
		eLiftIdle,
		eLift1,
		eLift2,
		eLift3,
		eLift4,
		eLift5,
		eLift6,
		eLift7,
		eLift8,
		eLift9
	};

	// Object pointers.
	Joystick*	m_pDriveController;	
	Joystick*	m_pAuxController;
	CDrive*		m_pDrive;
	CElevator*	m_pElevator;
	CArm*		m_pArm;
	CBucket*	m_pBucket;
	CVision*	m_pVision;
	Timer*		m_pTimer;
	LiveWindow* m_pLiveWindow;
	PowerDistributionPanel*		m_pPDP;

	// Initialize variables.
	int  	m_nTeleopState;
	int		m_nLiftState;
	double 	m_dDelayStartTime;
	bool 	m_bDriveControllerPOVUpPressed;
	bool 	m_bDriveControllerPOVDownPressed;
	bool	m_bDriveControllerPOVLeftPressed;
	bool	m_bDriveControllerPOVRightPressed;
	bool 	m_bDriveControllerButtonRBPressed;
	bool 	m_bDriveControllerButtonLBPressed;
	bool 	m_bDriveControllerButtonAPressed;
	bool 	m_bDriveControllerButtonBPressed;
	bool 	m_bDriveControllerButtonXPressed;
	bool 	m_bDriveControllerButtonYPressed;
	bool	m_bDriveControllerButtonLSPressed;
	bool	m_bDriveControllerButtonRSPressed;
	bool	m_bDriveControllerStartPressed;
	bool	m_bDriveControllerBackPressed;
	bool	m_bAuxControllerButtonAPressed;
	bool	m_bAuxControllerButtonBPressed;
	bool	m_bAuxControllerButtonXPressed;
	bool	m_bAuxControllerButtonYPressed;
	bool	m_bAuxControllerButtonRBPressed;
	bool	m_bAuxControllerButtonLBPressed;
	bool	m_bAuxControllerStartPressed;
	bool	m_bAuxControllerBackPressed;
	bool	m_bAuxControllerIsArmActive;
	bool	m_bAuxControllerButton11Pressed;
	bool	m_bAuxControllerButton12Pressed;
	bool	m_bAuxControllerStopAction;
	
};
////////////////////////////////////////////////////////////////////////////////
#endif