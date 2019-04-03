/****************************************************************************
	Description:	Defines the CCANPosition control class.

	Classes:		CCANPosition

	Project:		2019 DeepSpace Robot Code.

	Copyright 2019 First Team 3284 - Camdenton LASER Robotics.
****************************************************************************/
#ifndef CANPosition_h
#define CANPosition_h

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

using namespace frc;

// Default Constants set for drive motors.
const int		nCANPositionSlotIdx					= 		 0; 	// Which PID slot to pull gains from.  You can choose from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
const int 		nCANPositionPIDLoopIdx				=		 0; 	// Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. Just want the primary one.
const int 		nCANPositionTimeoutMs				= 		10; 	// Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails.
const int	 	nDefaultCANPositionPulsesPerRev		=  	  1024;		// Encoder Pulses Per Revolution (Quadrature).
const double 	dDefaultCANPositionRevsPerUnit		=    (1.000 / (5.875* 3.1415));	// Revolutions per unit of measure. (1 revs(Encoder)/(5.875 in * PI))
const double 	dDefaultCANPositionFwdHomeSpeed		=    0.000;		// Homing forward speed (set to zero because drive motors don't home)
const double 	dDefaultCANPositionRevHomeSpeed		=    0.000;		// Homing reverse speed (set to zero because drive motors don't home)
const double 	dDefaultCANPositionProportional		=    0.500; 	// Default proportional value.
const double 	dDefaultCANPositionIntegral			=    0.000;		// Default integral value.
const double 	dDefaultCANPositionDerivative		=    0.000;		// Default derivative value.
const double	dDefaultCANPositionFeedForward		=	 0.000;		// Default feed forward value.
const double 	dDefaultCANPositionVoltageRampRate	=    0.250;		// Default voltage ramp rate. This is in seconds from neutral to full output.
const double 	dDefaultCANPositionTolerance		=    0.250;		// Default tolerance in desired units.
const double 	dDefaultCANPositionLowerSoftLimit	= -250.000; 	// Default lower soft limit. This is in desired units.
const double 	dDefaultCANPositionUpperSoftLimit	=  250.000; 	// Default upper soft limit. This is in desired units.
const double 	dDefaultCANPositionIZone			=    5.000;		// Default IZone value. This is in the desired units.
const double 	dDefaultCANPositionMaxHomingTime	=    0.000;		// Default Maximum allowable time to home. Zero to disable timeout. This is in seconds.
const double 	dDefaultCANPositionMaxFindingTime	=    0.000;		// Default Maximum allowable time to move to position. Zero to disable timeout. This is in seconds.
const double	dDefualtCANPositionManualFwdSpeed 	=	 0.500;
const double	dDefualtCANPositionManualRevSpeed	=	-0.500;
enum State {eIdle, eHomingReverse, eHomingForward, eFinding, eManualForward, eManualReverse};
///////////////////////////////////////////////////////////////////////////////


/******************************************************************************
	Description:	CCANPosition control class.

	Argument:		None

	Derived From:	Nothing
******************************************************************************/
class CCANPosition
{
public:
	// Method prototypes.
	CCANPosition(int nDeviceID);
	~CCANPosition();

	void	Tick();
	void	SetSetpoint(double dPosition);
	double	GetSetpoint();
	void	StartHoming();
	void	Stop();
	void	SetTolerance(double dValue);
	double	GetTolerance();
	void	SetSoftLimits(double dMinValue, double dMaxValue);
	void	ConfigLimitSwitches(bool bFwdLimitNormallyOpen, bool bRevLimitNormallyOpen);
	void	SetAccumIZone(double dIZone);
	bool	IsFwdLimitSwitchPressed();
	bool	IsRevLimitSwitchPressed();
	bool	IsAtSetpoint();
	void	ResetEncoderPosition();
	void	SetPeakOutputPercent(double dMaxFwdOutput, double dMaxRevOutput);
	void	SetNominalOutputVoltage(double dNominalFwdOutput, double dNominalRevOutput);
	void	SetOpenLoopRampRate(double dOpenLoopRampRate);
	void	SetClosedLoopRampRate(double dClosedLoopRampRate);
	void	SetMotorNeutralMode(int nMode);
	double	GetActual();
	void	SetHomeSpeeds(double dFwdSpeed, double dRevSpeed);
	void	SetPulsesPerRev(int nPPR);
	void	SetRevsPerUnit(double dRPU);
	void	SetPIDValues(double dProportional, double dIntegral, double dDerivative, double dFeedForward = 0.000);
	void	SetMotorInverted(bool bInverted);
	void	SetSensorInverted(bool bInverted);
	void	ClearStickyFaults();
	void	SetManualSpeed(double dForward, double dReverse);

	WPI_TalonSRX*	GetMotorPointer()					{ return m_pMotor;														};
	bool	IsReady()									{ return m_bReady;														};
	bool	IsHomingComplete()							{ return m_bHomingComplete;												};
	void	SetMaxHomingTime(double dMaxHomingTime)		{ m_dMaxHomingTime = dMaxHomingTime;									};
	void	SetMaxFindingTime(double dMaxFindingTime)	{ m_dMaxFindingTime = dMaxFindingTime;									};
	State	GetState()									{ return m_nCurrentState;												};
	void	SetState(State nNewState)					{ m_nCurrentState = nNewState;											};
	double	GetMotorCurrent()							{ return m_pMotor->GetOutputCurrent();									};
	double	GetMotorVoltage()							{ return m_pMotor->GetMotorOutputVoltage(); 							};
	double	GetRevsPerUnit()							{ return m_dRevsPerUnit;												};
	int		GetPulsesPerRev()							{ return m_nPulsesPerRev;												};
	int		GetRawEncoderCounts()						{ return m_pMotor->GetSelectedSensorPosition(nCANPositionPIDLoopIdx);	};
	void	BackOffHome(bool bBackOff)					{ m_bBackOffHome = bBackOff;											};



private:
	// Object Pointers.
	WPI_TalonSRX*			m_pMotor;
	Timer*					m_pTimer;

	// Member Variables.
	bool					m_bFwdLimitSwitchNormallyOpen;
	bool					m_bRevLimitSwitchNormallyOpen;
	bool					m_bHomingComplete;
	bool					m_bReady;
	bool					m_bBackOffHome;
	int						m_nPulsesPerRev;
	double					m_dSetpoint;
	double					m_dRevsPerUnit;
	double					m_dFwdMoveSpeed;
	double					m_dRevMoveSpeed;
	double					m_dFwdHomeSpeed;
	double					m_dRevHomeSpeed;
	double					m_dTolerance;
	double					m_dLowerSoftLimit;
	double					m_dUpperSoftLimit;
	double					m_dIZone;
	double					m_dMaxHomingTime;
	double					m_dMaxFindingTime;
	double					m_dHomingStartTime;
	double					m_dFindingStartTime;
	State					m_nCurrentState;
	int						m_nDeviceID;
};
///////////////////////////////////////////////////////////////////////////////
#endif
