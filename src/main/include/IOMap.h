/****************************************************************************
	Description:	Defines the 2019 DeepSpace Robot I/O map.

	Classes:		None

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef IOMap_h
#define IOMap_h
/////////////////////////////////////////////////////////////////////////////

// Solenoid Channels.
const int nDriveShiftSolenoid			=  	0;		// Solenoid output for shifting gearbox   					(off = low gear)
const int nElevatorBrakeSolenoid		=	1;		// Solenoid output for Elevator brake						(off = engaged)
const int nArmBrakeSolenoid				=	2;		// Solenoid output for Arm brake							(off = engaged)
const int nBucketIntakeSolenoid			=	3;		// Solenoid output for Hatch intake mechanism				(off = disengaged)
const int nBucketLinearSolenoid			=	4;		// Solenoid output for moving the Hatch Intake mech.		(off = disengaged)
const int nBucketEjectSolenoid			=	5;		// Solenoid output for ejecting from the Hatch mech.		(off = disengaged)
const int nLiftLockSolenoid				=	6;		// Solenoid output for locking the lift until endgame.		(off = engaged)
const int nLiftShortSolenoid			=	7;		// Solenoid output for 1st HAB Zone lift.					(off = disengaged)
const int nLiftHighClimbSolenoid1		=	6;		// Solenoid output for 2nd HAB Zone lift.					(kFowrard)
const int nLiftHighClimbSolenoid2		=	7;		// Dumb solenoid output for 2nd HAB Zone lift.				(kReverse)

// CAN Device IDs.
const int nLeftDriveMotor1		  		=  	1;		// Spark Max ID for left drive motor 1
const int nLeftDriveMotor2		  		=  	2;		// Spark Max ID for left drive motor 2 						(Inverted output)
const int nLeftDriveMotor3		  		=   3;		// Spark Max ID for left drive motor 3
const int nRightDriveMotor1		  		=   4;		// Spark Max ID for right drive motor 4
const int nRightDriveMotor2		  		=   5;		// Spark Max ID for right drive motor 5						(Inverted output)
const int nRightDriveMotor3		  		=   6;		// Spark Max ID for right drive motor 6
const int nElevatorMotor1				=	7;		// Talon ID for elevator motor 1							(Inverted output)
const int nElevatorMotor2				=   8;		// Talon ID for elevator motor 2							(Inverted output)
const int nArmMotor						=   9;		// Talon ID for arm motor
const int nBucketIntakeMotor			=  10;		// Talon ID for intake roller motor
const int nLiftDriveMotor				=  11;		// Talon ID for lift driving motor
const int nPCM1							=  12;		// CAN ID for PCM 1.
const int nPCM2							=  13;		// CAN ID for PCM 2.

// PWM Channels.

// Relay Channels.

// Analog Channels.

// Digital Channels.
const int nCargoSensor					=	0;		// Digital port for Cargo Intake Sensor
const int nHatchSensor					=	1;		// Digital port for Hatch Intake Sensor
const int nDriveLeftClockA				=	2;		// Digital port for Left Drive Encoder, Clock A
const int nDriveLeftClockB				=	3;		// Digital port for Left Drive Encoder, Clock B
const int nDriveRightClockA				=	4;		// Digital port for Right Drive Encoder, Clock A
const int nDriveRightClockB				=	5;		// Digital port for Right Drive Encoder, Clock B
const int nLiftSensor					=	6;		// Digital port for Proximity Sensor for Lifting
const int nLiftStabilizerSensor			=	8;		// Digital port for Proximity Sensor for Stabilizer

// XBox Controller Button Assignments.
enum XBoxButtons 		{eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};
// Logitech Flight Stick Button Assignments.
enum LogButtons	 		{eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};
/////////////////////////////////////////////////////////////////////////////
#endif
