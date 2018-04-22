/*
 * elevator.cpp
 *
 *  Created on: Mar 26, 2018
 *      Author: James Kemp
 */


#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>
#include <buttons.h>
#include <elevator.h>

// 8000cnt / 32inch = 250 cnts/inch
// 12000 / 50inch = 240 cnts/inch

#define MAX_CNTS	16000
#define HOME_CNTS	200

#define HOME_LS		false	// Elevator home limit switch is active low.

int posSp = 0;		// This is the current position setpoint to the motor.
int posTar = 0;		// This position is where you want to be.


//	This zeros / homes the elevator and resets the encoder to zero.
// ============================================================================
bool resetElevatorPos( TalonSRX *mtr, DigitalInput *sw ) {
	static int cnt = 0;

	mtr->Set( ControlMode::PercentOutput, 0.0 );	// Let move coast down.
	posSp = HOME_CNTS;	posTar = HOME_CNTS;
	cnt += 1;

	// Make sure the limit switch is home AND it's been there for a bit.  That's to
	// make sure the elevator is fully down.
	if( sw->Get() == HOME_LS ) {
		if( cnt > 50 ) {
			mtr->SetSelectedSensorPosition( 0, 0, 0 );
			if( cnt == 51 ) printf( "Elev Homed\n" );
			return true;	// Return true that we've homed.
		}
	}
	else {
		cnt = 0;			// Keep counter / timer zeroed if elevator above switch.
		return false;		// Return false / elevator NOT homed.
	}

	// Should never get here.
	return false;		// Else, return false / elevator NOT homed.
}

// ============================================================================
void DriveElevator( double fMove, TalonSRX *mtr, DigitalInput *sw, struct btns *btns) {
	static bool flgStop = false;
	int rate = 0;
	static int cnt=0;

	// On button press, jump the target position to the given height.
	//if( btns->btnPress2[eHome2] ) setElevatorPos( 0 );
	//if( btns->btnPress2[eSwitch2] ) setElevatorPos( 4000 );
	//if( btns->btnPress2[eScale2] ) setElevatorPos( 8000 );

	// On neutral move, stop adjusting stuff!
	if( fabs(fMove) < 0.2 ) {		// Deadband of 20% on the move desire.
		if( flgStop == false) {
			flgStop = true;
			fMove = 0.0;
			posSp = posTar;
			mtr->SetIntegralAccumulator( 0.0, 0, kTO );
		}
	}
	else {
		flgStop = false;
		// fMove ranges from +1.0 to -1.0;
		if(fMove>=0){
			posTar += fMove * 200;	// Add or subtract to the desired position target.
		}else{
			posTar += fMove * 200;
		}

		if( posTar > MAX_CNTS )	 posTar = MAX_CNTS;	// Clamp target to acceptable range.
		if( posTar < HOME_CNTS ) posTar = HOME_CNTS;
		if( ++cnt > 20 ) {
			printf( "posTar: %d\n", posTar );
			cnt = 0;
		}
	}

	rate = fabs(fMove) * 250;	// Scale the rate of change into 0 to 150cnts per loop.
	if( rate < 25 ) rate = 50;	// Make sure a minimum is kept so we move a little.

	// Ramp "posSp" until it matches the target position.
	if( posSp + 100 < posTar )
		posSp += rate;
	else if ( posSp + 100 > posTar )
		posSp -= rate;
	else
		posSp = posTar;

	// Make sure 'posSp' (what we're going to give to the Talon as a position
	// setpoint) is in the acceptable range.
	if( posSp < HOME_CNTS ) posSp = HOME_CNTS;	// Min Sp / So it does not slam home.
	if( posSp > MAX_CNTS ) posSp = MAX_CNTS;	// Max Sp / So it does not over reach.

	// Drive motor in close-loop position mode.
	mtr->Set( ControlMode::Position, posSp );

	if( btns->btnPress2[ ten2 ] ) {
		printf( "Elev Sp vs Actual: %d    %d\n", posSp, mtr->GetSelectedSensorPosition(0) );
	}
}


//	Init the elevator system here.
// ============================================================================
void InitElevator( TalonSRX *mtr ) {

	mtr->NeutralOutput();
	mtr->ConfigOpenloopRamp( 0.2, kTO );
	mtr->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	mtr->SetSelectedSensorPosition( 0, 0, 0 );
	mtr->SetSensorPhase( true );
	mtr->SetInverted( true );
	mtr->ConfigClosedloopRamp( 0.25, kTO );
	mtr->ConfigForwardSoftLimitEnable( false, kTO );
	mtr->ConfigReverseSoftLimitEnable( false, kTO );

	// Close Loop Params for Position Control
	mtr->Config_kF(kPIDLoopIdx, 0.0,	kTO );	// Feed-Forward
	mtr->Config_kP(kPIDLoopIdx, 0.5,	kTO );	// P
	mtr->Config_kI(kPIDLoopIdx, 0.001,	kTO );	// I	~= P/100
	mtr->Config_kD(kPIDLoopIdx, 5.0,	kTO );	// D	~= P*10
	mtr->Config_IntegralZone( 0, 750,	kTO );	// I-Zone
	mtr->ConfigPeakOutputForward( 1.0,	kTO );	// Peak Forward
	mtr->ConfigPeakOutputReverse(-0.4,	kTO );	// Peak Reverse
}

//	Jump the target position to 'pos'.
// ============================================================================
void setElevatorPos( int pos ) {
	posTar = pos;
}
