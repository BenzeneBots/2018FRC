/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <ctre/Phoenix.h>
#include <WPILib.h>

//Add Timer Class
#include <Timer.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <Drive.h>

class Robot : public frc::IterativeRobot {
public:
	Drive *robotDrive;

	//Init Auton Timers
	Timer *autonTimer = new Timer();

	//Step Progression for Auton
	enum SampleAuton {driveStraight, finish};
	SampleAuton autonStatus = driveStraight;


	void RobotInit() {

		//initialize subsystems
		robotDrive = new Drive(2,1,4,3); // Our drive uses Talons 1,2,3,4
	}

	void AutonomousInit() override {
		//Sets first case for auton enums
		autonStatus = driveStraight;

		//Starts Timer for First Step
		autonTimer->Reset();
		autonTimer->Start();
	}

	void AutonomousPeriodic() {
		switch(autonStatus){
		case driveStraight:
			if(autonTimer->Get() < 5.0){ // Time needed to reach Auton Distance
				robotDrive->TankDrive(0.5,0.5);
			} else{
				robotDrive->TankDrive(0.0,0.0);
				autonStatus = finish;
			}
			break;
		case finish:
			robotDrive->TankDrive(0.0,0.0);
			break;
		default:
			robotDrive->TankDrive(0.0,0.0);
			break;
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic() {}

	void TestPeriodic() {}

private:

};

START_ROBOT_CLASS(Robot)
