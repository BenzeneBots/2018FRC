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

#define ELEVATOR_BOTTOM_HEIGHT -1200
#define ELEVATOR_SWITCH_HEIGHT 3467
#define ELEVATOR_SCALE_HEIGHT 13000

#define TURN_FACTOR 0.5
#define DRIVE_SPEED_FACTOR 0.9
#define DRIVE_SCALE 1.7
#define TURN_SCALE 1.1

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

	//Auton Stuff
	enum Steps {driveStraight, finished};
	Steps autonStatus = driveStraight;
	enum StepsCenterSwitch1Cube {driveStraight1,turn1,driveStraight2,turn2,driveStraight3,finished1};
	StepsCenterSwitch1Cube autonStatusCenterSwitch1Cube = driveStraight1;

	void RobotInit() {
		//populates auto chooser on dashboard
		m_chooser.AddDefault(CenterDriveStraight, CenterDriveStraight);
		m_chooser.AddObject(CenterSwitch1Cube, CenterSwitch1Cube);
		m_chooser.AddObject(CenterScale1Cube, CenterScale1Cube);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		//initialize subsystems
		robotDrive = new Drive(2,1,4,3,7); 			//drive uses Talons 1,2,3,4 and pigeonIMU port 7
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
		robotDrive->SetBrakeMode();
		autonStatus = driveStraight;


		robotDrive->ResetEncoders();

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
					if(gameData[0] == 'L')  {
					//If switch is on left

						switch(autonStatusCenterSwitch1Cube){
						case driveStraight1:
							if(AutonDriveStraight(12.0, robotDrive)){
								autonStatusCenterSwitch1Cube = turn1;
							}
							break;
						case turn1:
							if(AutonTurnLeft(60.0, robotDrive)){
								autonStatusCenterSwitch1Cube = driveStraight2;
							}
							break;
						case driveStraight2:
							if(AutonDriveStraight(24, robotDrive)){
								autonStatusCenterSwitch1Cube = turn2;
							}
							break;
						case turn2:
							if(AutonTurnRight(60.0, robotDrive)){
								autonStatusCenterSwitch1Cube = driveStraight3;
							}
							break;
						case driveStraight3:
							if(AutonDriveStraight(12.0, robotDrive)){
								autonStatusCenterSwitch1Cube = finished1;
							}
							break;
						case finished1:
							break;//do nothing
						default:
							break;//do nothing
						}
					} else {
					//If switch is on right

						switch(autonStatusCenterSwitch1Cube){
						case driveStraight1:
							if(AutonDriveStraight(12.0, robotDrive)){
								autonStatusCenterSwitch1Cube = turn1;
							}
							break;
						case turn1:
							if(AutonTurnRight(60.0, robotDrive)){
								autonStatusCenterSwitch1Cube = driveStraight2;
							}
							break;
						case driveStraight2:
							if(AutonDriveStraight(24, robotDrive)){
								autonStatusCenterSwitch1Cube = turn2;
							}
							break;
						case turn2:
							if(AutonTurnLeft(60.0, robotDrive)){
								autonStatusCenterSwitch1Cube = driveStraight3;
							}
							break;
						case driveStraight3:
							if(AutonDriveStraight(12.0, robotDrive)){
								autonStatusCenterSwitch1Cube = finished1;
							}
							break;
						case finished1:
							break;//do nothing
						default:
							break;//do nothing
								}
						  }
				    }


				}
			if (m_autoSelected == CenterScale1Cube) {
					// Custom Auto goes here
			}
			else {
				//Deafult Auto (DriveStraight)



				switch(autonStatus){
				case driveStraight:
					printf("DriveStraight\n");
					if(AutonDriveStraight(40.0, robotDrive)){
						autonStatus = finished;

					}
					break;

				case finished:
					robotDrive->TankDrive(0.0,0.0);
					break;//do nothing

				default:
					break;//do nothing
				}
			}
			//printf("RightDriveEnc %f \n",robotDrive->GetRightEncoderValue());
			//printf("LeftDriveEnc %f \n",robotDrive->GetLeftEncoderValue());
		}

	void TeleopInit() {
		//Initialize all the joysticks
		mainDriverStick = new Joystick(0);
		secondaryDriverStick = new Joystick(1);
		manipStick = new Joystick(2);

		//reset Drive Encoders
		robotDrive->ResetEncoders();

		robotDrive->SetCoastMode();
	}

	void TeleopPeriodic() {
		//drives robot according to joystick inputs
		double speedVal  = robotDrive->InputScale(DRIVE_SPEED_FACTOR * mainDriverStick->GetRawAxis(1), DRIVE_SCALE);

		//printf("rawVal: %f\n", -1.0 * driver)
		//printf("speedVal: %f\n", speedVal);

		double turnVal = robotDrive->InputScale(TURN_FACTOR * mainDriverStick->GetRawAxis(4), TURN_SCALE);
		robotDrive->ArcadeDrive(speedVal, turnVal);

		//drives elevator and updates sensor values
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
			robotElevator->SetElevatorTarget(ELEVATOR_SCALE_HEIGHT);
		}
		else if(manipStick->GetRawButton(9)||manipStick->GetRawButton(10)){
			robotElevator->SetElevatorTarget(ELEVATOR_SWITCH_HEIGHT);
		}
		else if(manipStick->GetRawButton(11) || manipStick->GetRawButton(12)){
			robotElevator->SetElevatorTarget(ELEVATOR_BOTTOM_HEIGHT);
		}
		robotElevator->MoveElevator(-1.0*manipStick->GetRawAxis(1));//updates elevator positions based on targets and joysticks

		//runs intake


		if(manipStick->GetRawButton(2)){
			robotIntake->OuttakeCubes();
		}
		else if(manipStick->GetRawButton(1)){
			robotIntake->IntakeCubes();
			printf("button 1 \n");
		}
		else{
			robotIntake->StopIntake();
		}

		//activates and deactivates claw
		if(manipStick->GetRawButton(6)){
			robotIntake->CloseClaw();
		}
		else if(manipStick->GetRawButton(5)){
			robotIntake->OpenClaw();
		}

		//stows and deploys intake
		if(manipStick->GetPOV(0) == 180){
			robotIntake->StowIntake();
			printf("Stowing Intake\n");
		}
		else if(manipStick->GetPOV(0) == 0){
			robotIntake->DeployIntake();
			printf("Deploying Intake\n");
		}



		//Prints some relevant stuff
		printf("ElevatorPos: %f \n", robotElevator->GetElevatorPosition());
		printf("RightDriveEnc %f \n",robotDrive->GetRightEncoderValue());
		printf("LeftDriveEnc %f \n",robotDrive->GetLeftEncoderValue());
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
