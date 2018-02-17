/*
 * Climber.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: Murali
 */

#include <Subsystems/Climber.h>
#include <WPILib.h>

#define CLIMBER_SPEED 0.5


Climber::Climber(int motorChannel) {
	// TODO Auto-generated constructor stub
	climberMotor = new Victor(motorChannel);
}

void Climber::SpoolClimber(bool state){
	double time = DriverStation::GetInstance().GetMatchTime();
	if(time < 40 && state){//little more than 30s to allow for field lag and stuff
		Climber::climberMotor->Set(CLIMBER_SPEED);
	}

}
