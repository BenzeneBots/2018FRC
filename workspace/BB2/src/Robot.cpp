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

class Robot : public frc::TimedRobot {
public:

	//Initialize the subsystem
	Intake::Intake *robotIntake;
	Drive::Drive *robotDrive;
	Elevator::Elevator *robotElevator;
	PigeonIMU *_pidgey;

	//init sensors
	DigitalInput *elevatorBottomSwitch;
	bool wasSwitchPressed;

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		//initialize subsystems
		robotDrive = new Drive::Drive(0,1,4,5); 			//drive uses Talons 0,1,2,3
		robotElevator = new Elevator::Elevator(4); 		//elevator uses Talon 4 and DIOs 0 and 1
		//robotElevator->SetEncoderPosition(0);
		_pidgey = new PigeonIMU(0);

		//initialize sensors
		elevatorBottomSwitch = new DigitalInput(0);
		wasSwitchPressed = 0;

		//reset Drive Encoders
		robotDrive->ResetEncoders();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		//reset Drive Encoders
		robotDrive->ResetEncoders();

		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//		 kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
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
		robotDrive->ArcadeDrive(-1.0*mainDriverStick->GetRawAxis(1), mainDriverStick->GetRawAxis(2));
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
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
