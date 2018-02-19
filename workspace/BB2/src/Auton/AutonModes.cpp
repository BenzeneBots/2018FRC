/*
 * AutonModes.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: Sanket Nayak
 */
#include <ctre/Phoenix.h>
#include <WPILib.h>

#include <Auton/AutonModes.h>


AutonModes::AutonModes() {
	// TODO Auto-generated constructor stub
}
void AutonModes::DriveStraight(double TargetDist,Drive *robotDrive, Auton *robotAuton){
	robotAuton->AutonDriveStraight(TargetDist,robotDrive);
}
void AutonModes::Center1CubeLeftSwitch(double TargetDist1,double TurnAng1, Drive *robotDrive, Auton *robotAuton){
	enum{Straight1,Reset,Turn1}currentState;
	currentState = Straight1;
	switch(currentState){
	case Straight1:
		robotAuton->AutonDriveStraight(TargetDist1,robotDrive);
		break;
	case Reset:
		robotAuton->Reset(robotDrive);
		break;
	case Turn1:
		robotAuton->AutonTurnLeft(TurnAng1,robotDrive);
	}
}
void AutonModes::Center1CubeRightSwitch(Auton *robotAuton){

}
void AutonModes::Center1CubeLeftScale(Auton *robotAuton){

}
void AutonModes::Center1CubeRightScale(Auton *robotAuton){

}
