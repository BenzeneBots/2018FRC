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
#include <Auton/Auton.h>


class Robot : public frc::TimedRobot {
public:

	//Initialize the subsystems

	Intake *robotIntake;
	Drive *robotDrive;
    Elevator *robotElevator;


	//init sensors
	DigitalInput *elevatorBottomSwitch;
	bool wasSwitchPressed;

	double temp;

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;

	void RobotInit() {
		//populates auto chooser on dashboard
		m_chooser.AddDefault(CenterDriveStraight, CenterDriveStraight);
		m_chooser.AddObject(CenterSwitch1Cube, CenterSwitch1Cube);
		m_chooser.AddObject(CenterScale1Cube, CenterScale1Cube);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		//initialize subsystems
		robotDrive = new Drive(1,2,3,4,7); 			//drive uses Talons 1,2,3,4 and pigeonIMU port 7
		robotElevator = new Elevator(5); 		//elevator uses Talon 5 and DIOs 0 and 1
		robotIntake = new Intake(0,1,0,1,2);		//Intake uses PWM 0 and 1, and PCM ports 0, 1, and 2


		//initialize sensors
		elevatorBottomSwitch = new DigitalInput(0);
		wasSwitchPressed = 0;

		//reset Encoders
		robotDrive->ResetEncoders();
		//robotElevator->SetEncoderPosition(0);
	}

	void AutonomousInit() {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString("Auto Selector",
		//		 kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == CenterSwitch1Cube) {
		}
		if (m_autoSelected == CenterScale1Cube) {

		}
		else {
		}
	}

	void AutonomousPeriodic() {
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if (m_autoSelected == CenterSwitch1Cube) {
			if(gameData.length() > 0)
			                {
					  if(gameData[0] == 'L')
					  {
						  //If switch is on left
						enum Steps {driveStraight1,turn1,driveStraight2,turn2,driveStraight3,finished};
						Steps autonStatus = driveStraight1;

						switch(autonStatus){
						case driveStraight1:
							if(AutonDriveStraight(12.0, robotDrive)){
								autonStatus = turn1;
							}
							break;
						case turn1:
							if(AutonTurnLeft(60.0, robotDrive)){
								autonStatus = driveStraight2;
							}
							break;
						case driveStraight2:
							if(AutonDriveStraight(24, robotDrive)){
								autonStatus = turn2;
							}
							break;
						case turn2:
							if(AutonTurnRight(60.0, robotDrive)){
								autonStatus = driveStraight3;
							}
							break;
						case driveStraight3:
							if(AutonDriveStraight(12.0, robotDrive)){
								autonStatus = finished;
							}
							break;
						case finished:
							break;//do nothing
						default:
							break;//do nothing
						}
					  } else {
							  //If switch is on right
							enum Steps {driveStraight1,turn1,driveStraight2,turn2,driveStraight3,finished};
							Steps autonStatus = driveStraight1;

							switch(autonStatus){
							case driveStraight1:
								if(AutonDriveStraight(12.0, robotDrive)){
									autonStatus = turn1;
								}
								break;
							case turn1:
								if(AutonTurnRight(60.0, robotDrive)){
									autonStatus = driveStraight2;
								}
								break;
							case driveStraight2:
								if(AutonDriveStraight(24, robotDrive)){
									autonStatus = turn2;
								}
								break;
							case turn2:
								if(AutonTurnLeft(60.0, robotDrive)){
									autonStatus = driveStraight3;
								}
								break;
							case driveStraight3:
								if(AutonDriveStraight(12.0, robotDrive)){
									autonStatus = finished;
								}
								break;
							case finished:
								break;//do nothing
							default:
								break;//do nothing
							}
					  }
			                }

				}
			}

			}
		if (m_autoSelected == CenterScale1Cube) {
				// Custom Auto goes here
		}
		else {
			//Deafult Auto (DriveStraight)
			enum Steps {driveStraight, finished};
			Steps autonStatus = driveStraight;

			switch(autonStatus){
			case driveStraight:
				if(AutonDriveStraight(10.0, robotDrive)){
					autonStatus = finished;
				}
				break;

			case finished:
				break;//do nothing

			default:
				break;//do nothing
			}
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
		double speedVal  = robotDrive->InputScale(-1.0 * mainDriverStick->GetRawAxis(1), 1.5);
		double turnVal = robotDrive->InputScale(mainDriverStick->GetRawAxis(2), 1.5);
		robotDrive->ArcadeDrive(speedVal, turnVal);

		//drives elevator and updates sensor values. Based on joystick, need to add preset buttons
		//toggles limitswitch value to see when it changes
		if((elevatorBottomSwitch->Get() == true) && (wasSwitchPressed == false)){//if the limit switch is being pressed for the first time, zero the encoder
			wasSwitchPressed = true;
			robotElevator->SetEncoderPosition(0);
		}
		else if((elevatorBottomSwitch->Get() == false) && (wasSwitchPressed == true)){//otherwise just change the variable so it doesn't screw up later
			wasSwitchPressed = false;
		}
		//Raises elevator to presets
		if(manipStick->GetRawButton(7)||manipStick->GetRawButton(8)){//top button = switch
			robotElevator->SetElevatorTarget(1.0);
		}
		else if(manipStick->GetRawButton(9)||manipStick->GetRawButton(10)){
			robotElevator->SetElevatorTarget(2.0);
		}
		else if(manipStick->GetRawAxis(11) || manipStick->GetRawAxis(12)){
			robotElevator->SetElevatorTarget(0.0);
		}
		robotElevator->MoveElevator(-1.0*manipStick->GetRawAxis(1));//updates elevator positions based on targets and joysticks

		//runs intake
		if(manipStick->GetPOV(0)){
			robotIntake->OuttakeCubes();
		}
		else if(manipStick->GetPOV(5)){
			robotIntake->IntakeCubes();
		}
		else{
			robotIntake->StopIntake();
		}

		//activates and deactivates claw
		if(manipStick->GetRawButton(3)){
			robotIntake->CloseClaw();
		}
		else if(manipStick->GetRawButton(4)){
			robotIntake->OpenClaw();
		}

		//stows and deploys intake
		if(manipStick->GetRawButton(5)){
			robotIntake->StowIntake();
		}
		else if(manipStick->GetRawButton(6)){
			robotIntake->DeployIntake();
		}



		//Prints some relevant stuff
		//printf("Right Drive: %f \n", robotDrive->GetRightEncoderValue());
		//printf("Left Drive: %f \n", robotDrive->GetLeftEncoderValue());
		//printf("Elevator Position: %f \n", robotElevator->GetElevatorPosition());
		printf("Raw Joystick: %f\n", -1.0 * mainDriverStick->GetRawAxis(1));
		printf("Processed Joystick: %f\n", speedVal);

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
