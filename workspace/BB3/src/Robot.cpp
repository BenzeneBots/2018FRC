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
#include <Victor.h>
#include <Drive/DifferentialDrive.h>



class Robot : public frc::TimedRobot {
public:

	//Drive Components
	DifferentialDrive *drivetrain;
	Victor *frontLeft, *frontRight, *rearLeft, *rearRight;

	//Joystick Values
	Joystick *driveStick;
	double speed = driveStick->GetRawAxis(1);
	double turn = driveStick->GetRawAxis(2);
	bool squaredInputs = true;

	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	};

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

	};

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	};

	void TeleopInit() {
		//Initialize motor controllers

		        // front set
		        //ctre::phoenix::motorcontrol::can::WPI_TalonSRX frontLeft(1);
		       // ctre::phoenix::motorcontrol::can::WPI_TalonSRX rearLeft(2);

				frc::Victor frontLeft(0);
				frc::Victor rearLeft(1);

		        // rear set
		        //ctre::phoenix::motorcontrol::can::WPI_TalonSRX frontRight(3);
		        //ctre::phoenix::motorcontrol::can::WPI_TalonSRX rearRight(4);

				frc::Victor frontRight(4);
				frc::Victor rearRight(5);

		        // speed controllers
		        SpeedControllerGroup leftDrive(frontLeft, rearLeft);
		        SpeedControllerGroup rightDrive{frontRight, rearRight};

		        // Create drive object
		       drivetrain = new DifferentialDrive (leftDrive, rightDrive);

		       // Use differential drive object
		        driveStick = new Joystick(1);
		        drivetrain->SetSafetyEnabled(true);


	};

	void TeleopPeriodic() {};

	void operatorControl() {
			while (IsOperatorControl() && IsEnabled()) {
				drivetrain->ArcadeDrive(speed,turn,squaredInputs);

				Wait(0.01);
			}
		}

	void TestPeriodic() {};

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

};

START_ROBOT_CLASS(Robot)
