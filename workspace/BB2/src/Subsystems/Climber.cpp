/*
 * Climber.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: Murali
 */

#include <Subsystems/Climber.h>
#include <WPILib.h>
#include <ctre/Phoenix.h>

#define CLIMBER_SPEED 0.45


Climber::Climber(int motorChannel) {
	// TODO Auto-generated constructor stub
	climberMotor = new Victor(motorChannel);
}

void Climber::SpoolClimber(bool state){
	//double time = DriverStation::GetInstance().GetMatchTime();
	//if(time < 40 && state){//little more than 30s to allow for field lag and stuff
	if(state) Climber::climberMotor->Set(CLIMBER_SPEED);
	else Climber::climberMotor->Set(0);
	//}
}
void Climber::RunClimber(){
	Climber::climberMotor->Set(0.5);
}
