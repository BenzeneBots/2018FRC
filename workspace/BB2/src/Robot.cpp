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

//include subsystems
#include <Subsystems/Intake.h>
#include <Subsystems/Drive.h>
#include <Subsystems/Elevator.h>
#include <Auton.cpp>



class Robot : public frc::TimedRobot {
public:

	//Initialize the subsystems

	AutonDrive *aDrive;// not used
	AutonElevator *aElevator; // not used
	AutonIntake *aIntake; // not used

	Intake *robotIntake;
	Drive *robotDrive;
    Elevator *robotElevator;
	PigeonIMU *_pidgey;

	//init sensors
	DigitalInput *elevatorBottomSwitch;
	bool wasSwitchPressed;

	double temp;

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;

	void RobotInit() {
		m_chooser.AddDefault(CenterDriveStraight, CenterDriveStraight);
		m_chooser.AddObject(CenterSwitch1Cube, CenterSwitch1Cube);
		m_chooser.AddObject(CenterScale1Cube, CenterScale1Cube);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		//initialize subsystems
		robotDrive = new Drive(1,2,3,4); 			//drive uses Talons 1,2,3,4
		robotElevator = new Elevator(5); 		//elevator uses Talon 5 and DIOs 0 and 1
		//robotElevator->SetEncoderPosition(0);
		_pidgey = new PigeonIMU(7);

		//initialize sensors
		elevatorBottomSwitch = new DigitalInput(0);
		wasSwitchPressed = 0;

		//reset Drive Encoders
		robotDrive->ResetEncoders();
	}

	void AutonomousInit() {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//		 kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == CenterSwitch1Cube) {
		}
		if (m_autoSelected == CenterSwitch1Cube) {
		}
		else {
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == CenterSwitch1Cube) {
					// Custom Auto goes here
				}
		if (m_autoSelected == CenterSwitch1Cube) {
			// Custom Auto goes here
			}
		else {
			AutonDriveStraight(24,robotDrive);
				}
	}

	void TeleopInit() {
		//Initialize all the joysticks
		mainDriverStick = new Joystick(0);
		secondaryDriverStick = new Joystick(1);
		manipStick = new Joystick(2);

		//reset Drive Encoders
		robotDrive->ResetEncoders();

	}

	void TeleopPeriodic() {
		//drives robot according to joystick inputs
		//robotDrive->ArcadeDrive(robotDrive->InputScale(-1.0*mainDriverStick->GetRawAxis(1),1.5), robotDrive->InputScale(mainDriverStick->GetRawAxis(2),1.5));
		robotDrive->ArcadeDrive(-1.0*mainDriverStick->GetRawAxis(1),mainDriverStick->GetRawAxis(2));
		printf("Elevator Position: %f \n", robotElevator->GetElevatorPosition());

		//drives elevator and updates sensor values. Based on joystick, need to add preset buttons
		robotElevator->SetToOutput(manipStick->GetRawAxis(1));//0.76 is optimal rate, ~9.12V (voltage control mode in Talon will be more consistent
		//toggles limitswitch value to see when it changes
		if((elevatorBottomSwitch->Get() == true) && (wasSwitchPressed == false)){//if the limit switch is being pressed for the first time, zero the encoder
			wasSwitchPressed = true;
			robotElevator->SetEncoderPosition(0);
		}
		else if((elevatorBottomSwitch->Get() == false) && (wasSwitchPressed == true)){//otherwise just change the variable so it doesn't screw up later
			wasSwitchPressed = false;

		}

		//Update Smart Dashboard
		frc::SmartDashboard::PutNumber("Elevator Encoder", robotElevator->GetElevatorPosition());


	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string CenterDriveStraight = "Center Line";
	const std::string CenterSwitch1Cube = "Center Switch";
	const std::string CenterScale1Cube = "Center Scale";

	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
