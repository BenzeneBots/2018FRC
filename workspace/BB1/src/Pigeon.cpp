//
//  Pigeon.cpp
// this code was written by FRC team 4384
//

#include "WPILib.h"
//#include "PigeonImu.h"
#include "Talon.h"

#include "Pigeon.h"

using namespace Pigeon;

//PigeonImu * pidgey = new PigeonImu(14);

int Pigeon::NormalizeAngle(int angle){
	while (angle < 0){
		angle += 360;
	}
	while(angle > 359){
		angle -= 360;
	}
	return angle;
}

void Pigeon::ResetYaw(){
	//pidgey->SetYaw(0.0);
}

double Pigeon::GetYaw(){
	//double ypr_array[3];
	//pidgey->GetYawPitchRoll(ypr_array);
	return 0;
}

double Pigeon::GetFusedHeading(){
	//return pidgey->GetFusedHeading();
	return 0;
}
