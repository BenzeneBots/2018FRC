/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <iostream>
#include <string>

#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Talon.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//include subsystems
#include <Drive.h>


#define ELEVATOR_BOTTOM_HEIGHT -1200
#define ELEVATOR_SWITCH_HEIGHT 3500	//TODO untested
#define ELEVATOR_SCALE_HEIGHT 13200 //untested

#define TURN_FACTOR 0.5
#define DRIVE_SPEED_FACTOR 1.0
#define RIGHT_DRIVE_CORRECTION 0.96
#define DRIVE_SCALE 1.7
#define TURN_SCALE 1.1

//TODO Find all the following distances/angles

//Auton Drive Straight Distance
#define CL_ZEROA 130.0

//Auton Center Distances
#define C1_ONEC 24.95
#define C2_ONEC 81.98
#define C3_ONEC 60.17

//Auton Center Angles
#define T1_ONEC 45.0
#define T2_ONEC 45.0

//Auton Sides Switch Distances
#define C1_SWITCH_ONES 166.57
#define C2_SWITCH_ONES 30.11

//Auton Sides Switch Angle
#define T1_SWITCH_ONES 90.0

//Auton Sides Scale Distances
#define C1_SCALE_ONES 322.13
#define C2_SCALE_ONES 116.88

//Auton Sides Scale Angle
#define T1_SCALE_ONES 90.0

//Auton Sides Far Cube Scale Distances
#define C1_ZEROS 234.08
#define C2_ZEROS 186.38
#define C3_ZEROS 63.11

//Auton Sides Far Cube Scale Angle
#define T1_ZEROS 90.0
#define T2_ZEROS 90.0
#define T3_ZEROS 30.0

class Robot : public frc::TimedRobot {
public:

	//Initialize the subsystems

	Drive *robotDrive;

	Timer *initTimer = new Timer();

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;

	//Step Progression for Auton
	enum SampleAuton {driveStraight, finish};
	SampleAuton autonStatus = driveStraight;

	void RobotInit() {

		//initialize subsystems
		robotDrive = new Drive(2,1,4,3); 			//drive uses Talons 1,2,3,4 and pigeonIMU port 0
		//robotElevator->SetEncoderPosition(0);
	}

	void AutonomousInit() {
		//Sets first case for auton enums
				autonStatus = driveStraight;

		//Starts Timer for First Step
		initTimer->Reset();
		initTimer->Start();

	}

	void AutonomousPeriodic() {
		switch(autonStatus){
			case driveStraight:
				if(initTimer->Get() < 5.0){ // Time needed to reach Auton Distance
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

	void TeleopInit() {
		//Initialize all the joysticks
		mainDriverStick = new Joystick(0);
		secondaryDriverStick = new Joystick(1);
		manipStick = new Joystick(2);

	}

	void TeleopPeriodic() {
		//drives robot according to joystick inputs
		double speedVal  = mainDriverStick->GetRawAxis(1);
		double turnVal =  mainDriverStick->GetRawAxis(4);

		robotDrive->ArcadeDrive(speedVal, turnVal);

	}

	void TestPeriodic() {}

private:

};

START_ROBOT_CLASS(Robot)
