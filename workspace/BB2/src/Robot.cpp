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

	Timer *initTimer = new Timer();

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;

	//Auton Stuff
	enum Steps {driveStraight0, finished};
	Steps autonStatusCrossLine = driveStraight0;
	enum StepsCenterSwitch1Cube {driveStraight1,turn1,driveStraight2,turn2,driveStraight3,setElevatorHeight0,moveElevator0,deploy0,outtake0,stow0,setElevatorHeight1,moveElevator1,finished1};
	StepsCenterSwitch1Cube autonStatusCenterSwitch1Cube = driveStraight1;

	void RobotInit() {
		//populates auto chooser on dashboard
		m_chooser.AddDefault(CenterDriveStraight, CenterDriveStraight);
		m_chooser.AddObject(CenterSwitch1Cube, CenterSwitch1Cube);
		m_chooser.AddObject(CenterScale1Cube, CenterScale1Cube);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		//initialize subsystems
		robotDrive = new Drive(2,1,4,3,0); 			//drive uses Talons 1,2,3,4 and pigeonIMU port 0
		robotElevator = new Elevator(5); 		//elevator uses Talon 5 and DIOs 0 and 1
		robotIntake = new Intake(0,1,0,1,2);		//Intake uses PWM 0 and 1, and PCM ports 0, 1, and 2


		//initialize sensors
		elevatorBottomSwitch = new DigitalInput(0);
		wasSwitchPressed = 0;

		//reset sensors
		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();
		robotDrive->ResetFusedHeading();
		//robotElevator->SetEncoderPosition(0);
	}

	void AutonomousInit() {
		m_autoSelected = m_chooser.GetSelected();
		//m_autoSelected = SmartDashboard::GetString("Auto Selector",
				 //CenterDriveStraight);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;
		robotDrive->SetBrakeMode();
		autonStatusCrossLine = driveStraight0;
		autonStatusCenterSwitch1Cube = deploy0; //TODO replace with driveStraight1


		//temporarily overrides DS to pick our own auton
		m_autoSelected = "CenterSwitch1Cube";

		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();

		if (m_autoSelected == "CenterSwitch1Cube") {
		}
		if (m_autoSelected == "CenterScale1Cube") {

		}
		else {

		}
	}

	void AutonomousPeriodic() {
			std::string gameData;
			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
			//printf("Encoder dist: %f\n", robotDrive->GetAverageEncoderDistance());
			//printf("Selected Auton %s\n",m_autoSelected.c_str());
			if (m_autoSelected == CenterSwitch1Cube) {
				if(gameData.length() > 0)
				                {
					if(gameData[0] == 'L')  {
					//If switch is on left

						switch(autonStatusCenterSwitch1Cube){
						case driveStraight1:
							if(AutonDriveStraight(32.0, robotDrive)){
								autonStatusCenterSwitch1Cube = turn1;
							}
							break;
						case turn1:
							if(AutonTurnLeft(90.0, robotDrive)){
								autonStatusCenterSwitch1Cube = driveStraight2;
							}
							break;
						case driveStraight2:
							if(AutonDriveStraight(45.0, robotDrive)){
								autonStatusCenterSwitch1Cube = turn2;
							}
							break;
						case turn2:
							if(AutonTurnRight(90.0, robotDrive)){
								autonStatusCenterSwitch1Cube = driveStraight3;
							}
							break;
						case driveStraight3:
							if(AutonDriveStraight(70.0, robotDrive)){
								autonStatusCenterSwitch1Cube = setElevatorHeight0;
							}
							break;
						case setElevatorHeight0:
							if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
								autonStatusCenterSwitch1Cube = moveElevator0;
							}
							break;
						case moveElevator0:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								autonStatusCenterSwitch1Cube = deploy0;
							}
							break;
						case deploy0:
							printf("Deploying Intake\n");
							if(AutonDeployIntake(robotIntake)){
								//Needed to initiate outtake and intake
								initTimer->Reset();
								initTimer->Start();

								autonStatusCenterSwitch1Cube = outtake0;

							}

							break;
						case outtake0:
							printf("Outtake Waiting Timer Value: %f\n", initTimer->Get());
							if(AutonOuttake(0.5, robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();
								autonStatusCenterSwitch1Cube = stow0;
							}
							break;
						case stow0:
							if(AutonStowIntake(robotIntake)){
								autonStatusCenterSwitch1Cube = setElevatorHeight1;
							}
							break;
						case setElevatorHeight1:
							if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
								autonStatusCenterSwitch1Cube = moveElevator1;
							}
							break;
						case moveElevator1:
							if(AutonMoveToHeight(robotElevator)){
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
						case setElevatorHeight0:
							if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
								autonStatusCenterSwitch1Cube = moveElevator0;
							}
							break;
						case moveElevator0:
							if(AutonMoveToHeight(robotElevator)){
								autonStatusCenterSwitch1Cube = deploy0;
							}
							break;
						case deploy0:
							if(AutonDeployIntake(robotIntake)){
								autonStatusCenterSwitch1Cube = outtake0;
								//Needed to initiate outtake and intake
								initTimer->Reset();
								initTimer->Start();
							}
							break;
						case outtake0:
							if(AutonOuttake(0.5,robotIntake) && initTimer->Get()>2.0){
								initTimer->Stop();
								autonStatusCenterSwitch1Cube = stow0;
							}
							break;
						case stow0:
							if(AutonStowIntake(robotIntake)){
								autonStatusCenterSwitch1Cube = setElevatorHeight1;
							}
							break;
						case setElevatorHeight1:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								autonStatusCenterSwitch1Cube = moveElevator1;
							}
							break;
						case moveElevator1:
							if(AutonMoveToHeight(robotElevator)){
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
			else if (m_autoSelected == CenterScale1Cube) {
					// Custom Auto goes here
			}
			else {
				//Default Auto (DriveStraight)
				switch(autonStatusCrossLine){
				case driveStraight0:
					if(AutonDriveStraight(130.0, robotDrive)){
						autonStatusCrossLine = finished;

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
		robotDrive->ResetYaw();
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
		//printf("EncoderDist: %f \n", robotDrive->GetAverageEncoderValue());
		printf("Yaw: %f\n", robotDrive->GetYaw());
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string CenterDriveStraight = "Center Line";
	const std::string CenterSwitch1Cube = "CenterSwitch1Cube";
	const std::string CenterScale1Cube = "CenterScale1Cube";

	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
