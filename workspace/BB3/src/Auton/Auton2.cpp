/*
 * Auton.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: Sanket Nayak
 */
#include <Auton/Auton2.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Timer.h>

#include <Auton/Auton.h>

#define ELEVATOR_BOTTOM_HEIGHT -600
#define ELEVATOR_SWITCH_HEIGHT 5500	//TODO untested
#define ELEVATOR_SCALE_HEIGHT 13200 //untested

Timer *auton2Timer = new Timer();

enum CurrentState{Step1,Step2,Step3,Step4,Step5,Step6,Step7,Step8,Step9,Step10} machineState;
enum ElevatorState{targetUp,moveUp,deploy,outtake,stow,targetDown,moveDown} elevatorState;
double startYaw;

void Run(Drive* robotDrive,Elevator* robotElevator,Intake*, bool state1,bool state2,bool state3,bool state4,bool state5,bool state6,bool state7,bool state8,bool state9,bool state10){
	switch(machineState){
	case Step1:
		if(state1){
			startYaw = robotDrive->GetYaw();
			machineState = Step2;
		}
		break;
	case Step2:
		if(state2){
			startYaw = robotDrive->GetYaw();
			machineState = Step3;
		}
		break;
	case Step3:
		if(state3){
			startYaw = robotDrive->GetYaw();
			machineState = Step4;
		}
		break;
	case Step4:
		if(state4){
			startYaw = robotDrive->GetYaw();
			machineState = Step5;
		}
		break;
	case Step5:
		if(state5){
			startYaw = robotDrive->GetYaw();
			machineState = Step6;
		}
		break;
	case Step6:
		if(state6){
			startYaw = robotDrive->GetYaw();
			machineState = Step7;
		}
		break;
	case Step7:
		if(state7){
				startYaw = robotDrive->GetYaw();
				machineState = Step8;
			}
		break;
	case Step8:
		if(state8){
			startYaw = robotDrive->GetYaw();
			machineState = Step9;
		}
		break;
	case Step9:
		if(state9){
			startYaw = robotDrive->GetYaw();
			machineState = Step10;
		}
		break;
	case Step10:
		if(state10){
			startYaw = robotDrive->GetYaw();
		}
		break;
	}
}
