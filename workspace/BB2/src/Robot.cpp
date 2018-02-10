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

	//Initialize all the subystems
	Intake::Intake *robotIntake;
	Drive::Drive *robotDrive;
	Elevator::Elevator *robotElevator;
	PigeonIMU *_pidgey;

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		//initialize subsystems
		robotDrive = new Drive::Drive(0,1,4,5); 			//drive uses Talons 0,1,2,3
		robotElevator = new Elevator::Elevator(4,5,4); 		//elevator uses Talon 4 and DIOs 4 and 5
		_pidgey = new PigeonIMU(0);


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
		mainDriverStick = new Joystick(1);
		secondaryDriverStick = new Joystick(2);
		manipStick = new Joystick(3);

		robotElevator->SetEncoderPosition(0);
		robotElevator->PIDInit();
	}

	void TeleopPeriodic() {
		//drives robot according to joystick inputs

		robotDrive->ArcadeDrive(-1.0*mainDriverStick->GetRawAxis(1), mainDriverStick->GetRawAxis(2));
		printf("Elevator Position: %f \n", robotElevator->GetElevatorPosition());
		robotElevator->EnableSoftLimits();
		robotElevator->MoveElevatorToSetPoint(manipStick->GetRawButton(1),manipStick->GetRawButton(2),manipStick->GetRawButton(3),manipStick->GetRawAxis(1));;

		//robotElevator->SetToOutput(manipStick->GetRawAxis(1));//0.76 is optimal rate, ~9.12V (voltage control mode in Talon will be more consistent

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
