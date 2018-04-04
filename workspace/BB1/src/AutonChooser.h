/*
 * AutonChooser.h
 *
 *  Created on: Apr 3, 2018
 *      Author: Sanket Nayak
 */

#ifndef AUTONCHOOSER_H_
#define AUTONCHOOSER_H_

#include <string>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>

#include <WPILib.h>

enum AutonPathId{
	CenterLeftSwitch,CenterRightSwitch,LeftSideSwitch,RightSideSwitch,LeftNearScale,
	RightNearScale,LeftFarScale,RightFarScale,DriveStraight,TestFunction
};

frc::LiveWindow& m_lw = *LiveWindow::GetInstance();

frc::SendableChooser<std::string> auton_chooser;
	const std::string DriveStraight0= "DriveStraight";
	const std::string CenterAuton = "CenterAuton";
	const std::string LeftAuton = "LeftAuton";
	const std::string RightAuton = "RightAuton";
	const std::string TestAuton = "TestAuton";

frc::SendableChooser<std::string> first_priority_chooser;
	const std::string Switch = "Switch";
	const std::string NearScale = "NearScale";
	const std::string FarScale = "FarScale";

frc::SendableChooser<std::string> second_priority_chooser;
	const std::string Switch1 = "Switch";
	const std::string NearScale1 = "NearScale";
	const std::string FarScale1 = "FarScale";
	const std::string DriveStraight1 = "DriveStraight";

frc::SendableChooser<std::string> third_priority_chooser;
	const std::string Switch2 = "Switch";
	const std::string NearScale2 = "NearScale";
	const std::string FarScale2 = "FarScale";
	const std::string DriveStraight2 = "DriveStraight";

std::string autoSelected = auton_chooser.GetSelected();
std::string	firstPrioritySelected = first_priority_chooser.GetSelected();
std::string	secondPrioritySelected = second_priority_chooser.GetSelected();
std::string	thirdPrioritySelected = third_priority_chooser.GetSelected();

void AutonDashboard(){
	//populates auto chooser on dashboard
	auton_chooser.AddDefault(DriveStraight0, DriveStraight0);
	auton_chooser.AddObject(CenterAuton, CenterAuton);
	auton_chooser.AddObject(LeftAuton, LeftAuton);
	auton_chooser.AddObject(RightAuton, RightAuton);
	auton_chooser.AddObject(TestAuton,TestAuton);

	first_priority_chooser.AddDefault(NearScale, NearScale);
	first_priority_chooser.AddObject( Switch, Switch);
	first_priority_chooser.AddObject(FarScale,FarScale);

	second_priority_chooser.AddDefault(FarScale1, FarScale1);
	second_priority_chooser.AddObject( Switch1, Switch1);
	second_priority_chooser.AddObject(NearScale1,NearScale1);
	second_priority_chooser.AddObject(DriveStraight1,DriveStraight1);

	third_priority_chooser.AddDefault( Switch2, Switch2);
	third_priority_chooser.AddObject(FarScale2, FarScale2);
	third_priority_chooser.AddObject(NearScale2,NearScale2);
	third_priority_chooser.AddObject(DriveStraight2,DriveStraight2);

	frc::SmartDashboard::PutData("Auton Modes", &auton_chooser);
	frc::SmartDashboard::PutData("Auton 1st Priority", &first_priority_chooser);
	frc::SmartDashboard::PutData("Auton 2nd Priority", &second_priority_chooser);
	frc::SmartDashboard::PutData("Auton 3rd Priority", &third_priority_chooser);
}

int ChooseAuton(std::string gameData){
	if(gameData.length()>0){//if the auton data exists
		if(autoSelected == CenterAuton){//if center auto is selected
			if(gameData[0] == 'L'){//if the switch is on the left run the center left auto
				return CenterLeftSwitch;
			}
			else{ //if the switch is on the right run the center right auto
				return CenterLeftSwitch;
			}
		}//end CenterAuton logic

		if(autoSelected == LeftAuton){//if left auto is selected
			if(firstPrioritySelected == Switch){
				if(gameData[0] == 'L'){//if the switch is on the left run the center left auto
					return LeftSideSwitch;
				}
				else{
					if(secondPrioritySelected == NearScale1){
						if(gameData[1] == 'L'){
							return LeftNearScale;
						}
						else{
							if(thirdPrioritySelected == FarScale2){
								return LeftFarScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else if(secondPrioritySelected == FarScale1){
						if(gameData[1] == 'R'){
							return LeftFarScale;
						}
						else{
							if(thirdPrioritySelected == NearScale2){
								return LeftNearScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else{
						return DriveStraight;
					}
				}
			}
			else if(firstPrioritySelected == NearScale){
				if(gameData[1] == 'L'){
					return LeftNearScale;
				}
				else{
					if(secondPrioritySelected == FarScale1){
						if(gameData[1] == 'R'){
							return LeftFarScale;
						}
						else{
							if(thirdPrioritySelected == Switch2){
								return LeftSideSwitch;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else if(secondPrioritySelected == Switch1){
						if(gameData[0] == 'L'){
							return LeftNearScale;
						}
						else{
							if(thirdPrioritySelected == FarScale2){
								return LeftFarScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else{
						return DriveStraight;
					}
				}
			}
			else if(firstPrioritySelected==FarScale){
				if(gameData[1] == 'R'){
					return LeftFarScale;
				}
				else{
					if(secondPrioritySelected == NearScale1){
						if(gameData[1]=='L'){
							return LeftNearScale;
						}
						else{
							if(thirdPrioritySelected == Switch2){
								return LeftSideSwitch;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else if(secondPrioritySelected == Switch1){
						if(gameData[0]=='L'){
							return LeftSideSwitch;
						}
						else{
							if(thirdPrioritySelected == NearScale2){
								return LeftNearScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else{
						return DriveStraight;
					}
				}
			}
			else{
				return DriveStraight;
			}
		}

		if(autoSelected == RightAuton){//if right auto is selected
			if(firstPrioritySelected == Switch){
				if(gameData[0] == 'R'){//if the switch is on the Right run the center Right auto
					return RightSideSwitch;
				}
				else{
					if(secondPrioritySelected == NearScale1){
						if(gameData[1] == 'R'){
							return RightNearScale;
						}
						else{
							if(thirdPrioritySelected == FarScale2){
								return RightFarScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else if(secondPrioritySelected == FarScale1){
						if(gameData[1] == 'L'){
							return RightFarScale;
						}
						else{
							if(thirdPrioritySelected == NearScale2){
								return RightNearScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else{
						return DriveStraight;
					}
				}
			}
			else if(firstPrioritySelected == NearScale){
				if(gameData[1] == 'R'){
					return RightNearScale;
				}
				else{
					if(secondPrioritySelected == FarScale1){
						if(gameData[1] == 'L'){
							return RightFarScale;
						}
						else{
							if(thirdPrioritySelected == Switch2){
								return RightSideSwitch;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else if(secondPrioritySelected == Switch1){
						if(gameData[0] == 'R'){
							return RightNearScale;
						}
						else{
							if(thirdPrioritySelected == FarScale2){
								return RightFarScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else{
						return DriveStraight;
					}
				}
			}
			else if(firstPrioritySelected==FarScale){
				if(gameData[1] == 'L'){
					return RightFarScale;
				}
				else{
					if(secondPrioritySelected == NearScale1){
						if(gameData[1]=='R'){
							return RightNearScale;
						}
						else{
							if(thirdPrioritySelected == Switch2){
								return RightSideSwitch;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else if(secondPrioritySelected == Switch1){
						if(gameData[0]=='R'){
							return RightSideSwitch;
						}
						else{
							if(thirdPrioritySelected == NearScale2){
								return RightNearScale;
							}
							else{
								return DriveStraight;
							}
						}
					}
					else{
						return DriveStraight;
					}
				}
			}
			else{
				return DriveStraight;
			}
		}

		else if(autoSelected == DriveStraight0){//if drive straight is selected
			return DriveStraight;
		}
		else if(autoSelected == TestAuton){
			return TestFunction;
		}
	}
	else{ //if no gama data is received
		return DriveStraight;
	}
}
#endif /* AUTONCHOOSER_H_ */
