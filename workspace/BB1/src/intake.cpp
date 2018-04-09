/*
 * intake.cpp
 *
 *  Created on: Apr 5, 2018
 *      Author: Sabita Dhal
 */

#include <intake.h>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Talon.h>

bool intakeDeployedStatus, clawOpenStatus;
Timer *deployTimer;
Timer *clawTimer;
bool deployStatus;
double midStatus;

// ============================================================================
void IntakeInit( DoubleSolenoid *cp ) {
	//Sets initial positions
	if(cp->Get() == frc::DoubleSolenoid::Value::kForward) intakeDeployedStatus = true;
	else intakeDeployedStatus = false;

	clawOpenStatus = false;

	cp->Set(DoubleSolenoid::kOff);
	deployStatus = true;				// Not used - should be deleted.
	midStatus = 0;

	deployTimer = new Timer();
	clawTimer = new Timer();
}

// ============================================================================
void CheesyIntake(Victor *it, double ejectSp, double intakeSp, bool intake2, bool eject2 ) {
	static bool wasIntaking = false;
	// Run intake motor according to the buttons.
	if ( ejectSp > 0.05 || eject2) {
			if( ejectSp < 0.05 ) it->Set( 1.0 );
			else it->Set( ejectSp );
			wasIntaking = false;
	}
	else if( intakeSp > 0.05 || intake2 ){
		if( intakeSp < 0.05 ) it->Set( -1.0 );
		else it->Set( intakeSp * -1.0 );
		wasIntaking = true;
	}
	else if( wasIntaking ) {
		it->Set( -0.13 );
	}
	else{
		it->Set( 0.0 );
	}
}

// ============================================================================
void OpenClaw( Solenoid* clawActuator ) {

	if(intakeDeployedStatus){//only open claw if intake is deployed
		clawActuator->Set(true);
		clawOpenStatus = true;
		printf("Opening Claw \n");
	}
}

// ============================================================================
void CloseClaw( Solenoid* clawActuator ) {

	clawActuator->Set(false);//since extended cylinder is closed claw
	clawOpenStatus = false;
	printf( "Closing Claw \n" );
}

// ============================================================================
void DeployIntake( DoubleSolenoid* angleActuator ) {

	angleActuator->Set(frc::DoubleSolenoid::Value::kForward);
	intakeDeployedStatus = true;
}


// ============================================================================
void StowIntake( Solenoid* clawActuator, DoubleSolenoid* angleActuator ) {

	if(clawOpenStatus){//if claw is open, close it before stowing
		CloseClaw(clawActuator);
	}

	//now that we are sure intake is closed, proceed with stowing
	angleActuator->Set(frc::DoubleSolenoid::Value::kReverse);
	intakeDeployedStatus = false;
}

// ============================================================================
void BenzeneIntake( int joyPOV, Solenoid* clawActuator, DoubleSolenoid* angleActuator, Victor *it, double ejectSp, double intakeSp, bool intake2, bool eject2, bool openClaw, bool closeClaw ) {

	if( midStatus == 1 ) {
		CloseClaw(clawActuator);
	}

	if( openClaw ) {
		OpenClaw( clawActuator );
	}
	else if( closeClaw ){
		CloseClaw( clawActuator );
	}


	CheesyIntake( it, ejectSp, intakeSp, intake2, eject2 );

	if( joyPOV == 180 ) { //Claw is stowed
		midStatus = 0;
		clawTimer->Reset();
		clawTimer->Start();
		intakeDeployedStatus = false;
	}
	else if( joyPOV == 90 ) { //Claw is Middle Position 1
		midStatus = 1;
		clawTimer->Reset();
		clawTimer->Start();
	}
	else if( joyPOV == 270 ) { //Claw is Middle Position 1
		midStatus = 1;
		clawTimer->Reset();
		clawTimer->Start();
	}
	else if( joyPOV == 0 ) { //Claw is deployed
		midStatus = 0;
		clawTimer->Reset();
		clawTimer->Start();
		intakeDeployedStatus = true;
	}

	if(intakeDeployedStatus){
		if(midStatus == 0 ) {
			DeployIntake(angleActuator);
		}
		else if( midStatus == 1 ) { //TODO Tune this
			if( clawTimer->Get() <= 1.1 ) {
				angleActuator->Set( DoubleSolenoid::kReverse );
			}else{
				angleActuator->Set( DoubleSolenoid::kOff );
			}
		}
		else if( midStatus == 2 ) { //TODO Tune this
			if(clawTimer->Get() <= 0.8){
				angleActuator->Set( DoubleSolenoid::kReverse );
			}else{
				angleActuator->Set( DoubleSolenoid::kOff );
			}
		}
	}
	else{
		if( midStatus == 0){
			StowIntake( clawActuator,angleActuator );
		}
		else if( midStatus == 1){ //TODO Tune this
			if(clawTimer->Get() <= 0.285){
				angleActuator->Set( DoubleSolenoid::kForward );
			}else{
				angleActuator->Set( DoubleSolenoid::kOff );
			}
		}
		else if( midStatus == 2 ) { //TODO Tune this
			if( clawTimer->Get() <= 0.2 ) {
				angleActuator->Set( DoubleSolenoid::kForward );
			} else {
				angleActuator->Set( DoubleSolenoid::kOff );
			}
		}
	}
}
