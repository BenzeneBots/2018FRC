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
#include <Subsystems/Intake.h>
#include <Subsystems/Drive.h>
#include <Subsystems/Elevator.h>
#include <Subsystems/Climber.h>

//include all autons
#include <Auton/AutoCommand.h>
#include <Auton/AutonDeployIntake.h>
#include <Auton/SequentialCommand.h>
#include <Auton/AutonDriveStraight.h>
#include <Auton/AutonIntake.h>
#include <Auton/AutonMoveElevatorToHeight.h>
#include <Auton/AutonOuttake.h>
#include <Auton/AutonStowIntake.h>
#include <Auton/AutonTurnLeft.h>
#include <Auton/AutonTurnRight.h>
#include <Auton/MotionMagicStraight.h>
#include <Auton/AutonPathfinder.h>


#define ELEVATOR_BOTTOM_HEIGHT -600
#define ELEVATOR_SWITCH_HEIGHT 5500	//TODO untested
#define ELEVATOR_SCALE_HEIGHT 13200 //untested

#define TURN_FACTOR 0.5
#define DRIVE_SPEED_FACTOR 1.0
#define RIGHT_DRIVE_CORRECTION 0.96
#define DRIVE_SCALE 1.7
#define TURN_SCALE 1.1

//TODO Find all the following distances/angles

/* Auton Naming Rules:
	 * c = constant speed forward driving
	 * t = turn
	 * e = set elevator target
	 * m = move elevator
	 * o = outtake
	 * d = deploy
	 * i = intake
	 * s = stow
	 * fin = finish
	 *
	 * Zero = No-Cube
	 * One = One-Cube
	 * Two = Two-Cube
	 *
	 * A = Drive from any position
	 * C = Center
	 * S = Sides
	 *
	 * # - number of times mentioned in enum
	 *
	 *
	 *
	 * Example:
	 * c1_OneC = first time driving forward on One-cube Center Auton
	 */

//Auton Drive Straight Distance
#define CL_ZEROA 170.0

//Auton Center Distances
#define C1_ONEC 30.42
#define C2_ONEC 46.30
#define C3_ONEC 61.0

//Auton Center Angles
#define T1_ONEC 90.0
#define T2_ONEC 90.0

//Auton Sides Switch Distances
#define C1_SWITCH_ONES 126.16
#define C2_SWITCH_ONES 3.0

//Auton Sides Switch Angle
#define T1_SWITCH_ONES 70.0

//Auton Sides Scale Distances //TODO Change to 235.00
#define C1_SCALE_ONES 235.0
#define C2_SCALE_ONES 12.0 //TODO TUNE THIS

//Auton Sides Scale Angle
#define T1_SCALE_ONES 30.0

//Auton Sides Far Cube Scale Distances
#define C1_ZEROS 228.08
#define C2_ZEROS 237.38
#define C3_ZEROS 75.11

//Auton Sides Far Cube Scale Angle
#define T1_ZEROS 90.0
#define T2_ZEROS 90.0
#define T3_ZEROS 90.0

class Robot : public frc::TimedRobot {
public:

	//Initialize the subsystems

	Intake *robotIntake;
	Drive *robotDrive;
	double driveRevFactor = -1.0;
	bool wasButtonPressed = false;
	bool wasIntaking = false;
	bool enableScaleClose = true;
	bool enableScaleFar = true;
    Elevator *robotElevator;
    Climber *robotClimber;

	//init sensors
	DigitalInput *elevatorBottomSwitch;
	bool wasSwitchPressed;
	bool driveDirection;

	//used to make Smart Dash Icons
	bool leftCloseSwitchDash;
	bool rightCloseSwitchDash;
	bool leftScaleDash;
	bool rightScaleDash;
	bool leftFarSwitchDash;
	bool rightFarSwitchDash;


	double startYaw;
	int firstPriority;
	int secondPriority;
	double matchTime;

	SequentialCommand* mainAutoCommand = NULL;
	SequentialCommand* elevatorSwitchCommand;
	SequentialCommand* elevatorScaleCommand;

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;


	void RobotInit() {
		//Initialize all the joysticks
		mainDriverStick = new Joystick(0);
		secondaryDriverStick = new Joystick(1);
		manipStick = new Joystick(2);

		//populates auto chooser on dashboard
		auton_chooser.AddDefault(DriveStraight, DriveStraight);
		auton_chooser.AddObject(Center1Cube, Center1Cube);
		auton_chooser.AddObject(Left1Cube, Left1Cube);
		auton_chooser.AddObject(Right1Cube, Right1Cube);

		priority_chooser.AddDefault(Switch , Switch);
		priority_chooser.AddObject( Scale , Scale);

		frc::SmartDashboard::PutData("Auton Modes", &auton_chooser);
		frc::SmartDashboard::PutData("Auton Priority", &priority_chooser);

		DriverStation::Alliance color = DriverStation::GetInstance().GetAlliance();

		if(color == DriverStation::Alliance::kBlue){
			frc::SmartDashboard::PutString("Alliance Color", "Blue");
		}else{
			frc::SmartDashboard::PutString("Alliance Color", "Red");
		}

		//Camera Stuff
		 CameraServer::GetInstance()->StartAutomaticCapture();

		//initialize subsystems
		robotDrive = new Drive(2,1,4,3,0); 			//drive uses Talons 1,2,3,4 and pigeonIMU port 0
		robotElevator = new Elevator(5); 		//elevator uses Talon 5 and DIOs 0 and 1
		robotIntake = new Intake(0,1,0,1,2);		//Intake uses PWM 0 and 1, and PCM ports 0, 1, and 2
		robotClimber = new Climber(8);			//climber uses PWM 2


		//initialize sensors
		elevatorBottomSwitch = new DigitalInput(0);
		wasSwitchPressed = 0;

		//reset sensors
		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();
		robotDrive->ResetFusedHeading();
		//robotElevator->SetEncoderPosition(0);

		//Load Waypoints into Memory
		robotDrive->Load_Waypoints();
	}

	void AutonomousInit() {
		m_autoSelected = auton_chooser.GetSelected();
		m_prioritySelected = priority_chooser.GetSelected();
		std::cout << "Auto selected: " << m_autoSelected << std::endl;
		std::cout << "Priority selected: " << m_prioritySelected << std::endl;

		if(m_prioritySelected == "Switch"){
			firstPriority = 0;
			secondPriority = 1;
		} else{
			firstPriority = 1;
			secondPriority = 0;
		}

		robotDrive->SetBrakeMode();

		//temporarily overrides DS to pick our own auton
			//m_autoSelected = "DriveStraight";
			//m_autoSelected = "Center1Cube";
			//m_autoSelected = "Right1Cube";
			//m_autoSelected = "Left1Cube"

		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();
		startYaw = robotDrive->GetYaw();

		elevatorSwitchCommand = AUTO_SEQUENTIAL(
				new AutonMoveElevatorToHeight(robotElevator, ELEVATOR_SWITCH_HEIGHT),
				new AutonDeployIntake(robotIntake),
				new AutonOuttake(robotIntake, 1.5),
				new AutonStowIntake(robotIntake),
				new AutonMoveElevatorToHeight(robotElevator, ELEVATOR_BOTTOM_HEIGHT));

		elevatorScaleCommand = AUTO_SEQUENTIAL(
						new AutonMoveElevatorToHeight(robotElevator, ELEVATOR_SCALE_HEIGHT),
						new AutonDeployIntake(robotIntake),
						new AutonOuttake(robotIntake, 1.5),
						new AutonStowIntake(robotIntake),
						new AutonMoveElevatorToHeight(robotElevator, ELEVATOR_BOTTOM_HEIGHT));


		std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData.length()>0){//if the auton data exists
			if(m_autoSelected == Center1Cube){//if center auton is selected

				if(gameData[0] == 'L'){//creates left switch from center auton
					mainAutoCommand = AUTO_SEQUENTIAL(
							new AutonDriveStraight(robotDrive, C1_ONEC),
							new AutonTurnLeft(robotDrive, T1_ONEC),
							new AutonDriveStraight(robotDrive, C2_ONEC),
							new AutonTurnRight(robotDrive, T2_ONEC),
							new AutonDriveStraight(robotDrive, C3_ONEC),
							elevatorSwitchCommand);
				}
				else{//otherwise creates right switch from center auton
					mainAutoCommand = AUTO_SEQUENTIAL(
							new AutonDriveStraight(robotDrive, C1_ONEC),
							new AutonTurnRight(robotDrive, T1_ONEC),
							new AutonDriveStraight(robotDrive, C2_ONEC),
							new AutonTurnLeft(robotDrive, T2_ONEC),
							new AutonDriveStraight(robotDrive, C3_ONEC),
							elevatorSwitchCommand);
				}
			}

			else if(m_autoSelected == Left1Cube){//if cube auton from left is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'L'){//if scale is on left go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new AutonDriveStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnRight(robotDrive, T1_SCALE_ONES),
								new AutonDriveStraight(robotDrive, C2_SCALE_ONES),
								elevatorScaleCommand);
					}
					else{//otherwise go for right scale but don't drop cube
						mainAutoCommand = AUTO_SEQUENTIAL(
								new AutonDriveStraight(robotDrive, C1_ZEROS),
								new AutonTurnRight(robotDrive, T1_ZEROS),
								new AutonDriveStraight(robotDrive, C2_ZEROS),
								new AutonTurnLeft(robotDrive, T2_ZEROS));
					}

				}
				else{//if priority is switch
					if(gameData[0] == 'L'){//if switch is on left go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new AutonDriveStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnRight(robotDrive, T1_SWITCH_ONES),
								new AutonDriveStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for right switch
						//TODO add right switch opposite auton
					}
				}
			}

			else if(m_autoSelected == Right1Cube){//if cube auton from right is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'R'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new AutonDriveStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnLeft(robotDrive, T1_SCALE_ONES),
								new AutonDriveStraight(robotDrive, C2_SCALE_ONES),
								elevatorScaleCommand);
					}
					else{//otherwise go for opposite scale but don't drop the cube
						mainAutoCommand = AUTO_SEQUENTIAL(
								new AutonDriveStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft(robotDrive, T1_ZEROS),
								new AutonDriveStraight(robotDrive, C2_ZEROS),
								new AutonTurnRight(robotDrive, T2_ZEROS));
					}

				}
				else{//if priority is switch
					if(gameData[0] == 'R'){//if switch is on right side, go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new AutonDriveStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnLeft(robotDrive, T1_SWITCH_ONES),
								new AutonDriveStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for left switch
						//TODO add left switch opposite auton
					}
				}
			}

			else{//defaults to driveStraight/crossLine auton
				mainAutoCommand = AUTO_SEQUENTIAL(
						//new AutonDriveStraight(robotDrive, 60));
						new AutonPathfinder(robotDrive,0,false));
			}
		}
		else {//if the game data doesn't exist for some reason default to crossing the line
			mainAutoCommand = AUTO_SEQUENTIAL(
					new AutonDriveStraight(robotDrive, CL_ZEROA));
		}

		if(mainAutoCommand) mainAutoCommand->Initialize();



	}

	void AutonomousPeriodic() {
		if(mainAutoCommand) mainAutoCommand->Run();
	}

	void TeleopInit() {

		//reset Drive Encoders
		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();
		robotDrive->SetBrakeMode();

		//Switch/Scale Array Dashboard Display
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData.length() > 0){
			if(gameData[0] == 'L')  {
				leftCloseSwitchDash = true;
				rightCloseSwitchDash = false;
			}else{
				leftCloseSwitchDash = false;
				rightCloseSwitchDash = true;
			}
			if(gameData[1] == 'L')  {
				leftScaleDash = true;
				rightScaleDash = false;
			}else{
				leftScaleDash = false;
				rightScaleDash = true;
			}
			if(gameData[2] == 'L')  {
				leftFarSwitchDash = true;
				rightFarSwitchDash = false;
			}else{
				leftFarSwitchDash = false;
				rightFarSwitchDash = true;
			}
		}else{
			leftCloseSwitchDash = false;
			rightCloseSwitchDash = false;
			leftScaleDash = false;
			rightScaleDash = false;
			leftFarSwitchDash = false;
			rightFarSwitchDash = false;
		}
		frc::SmartDashboard::PutBoolean("Switch 1 Left", leftCloseSwitchDash);
		frc::SmartDashboard::PutBoolean("Switch 1 Right", rightCloseSwitchDash);
		frc::SmartDashboard::PutBoolean("Scale Left", leftScaleDash);
		frc::SmartDashboard::PutBoolean("Scale Right", rightScaleDash);
		frc::SmartDashboard::PutBoolean("Switch 2 Left", leftFarSwitchDash);
		frc::SmartDashboard::PutBoolean("Switch 2 Right", rightFarSwitchDash);

	}

	void TeleopPeriodic() {
		printf("Teleop \n");

		matchTime = DriverStation::GetInstance().GetMatchTime();
		frc::SmartDashboard::PutNumber("Match Time", matchTime);

		//Dashboard Display for Drive Direction
		if(driveRevFactor == 1.0){
			driveDirection = true;
		} else{
			driveDirection = false;
		}
		frc::SmartDashboard::PutBoolean("Drive Direction",driveDirection);


		//drives robot according to joystick inputs

		//updates drive direction
		if(mainDriverStick->GetRawButton(6) && !wasButtonPressed){ //if this button is pressed for the first time:
			wasButtonPressed = true;
			driveRevFactor *= -1.0; //reverse drive direction
		}
		else if(!mainDriverStick->GetRawButton(6)){
			wasButtonPressed = false;
		}

		double throttle =driveRevFactor * mainDriverStick->GetRawAxis(1);
		robotDrive->BenzeneDrive(throttle, mainDriverStick->GetRawAxis(2), mainDriverStick->GetRawButton(2));

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
		if(manipStick->GetRawButton(7)){//top button = switch
			robotElevator->SetElevatorTarget(ELEVATOR_SCALE_HEIGHT);
		}
		else if(manipStick->GetRawButton(9)){
			robotElevator->SetElevatorTarget(ELEVATOR_SWITCH_HEIGHT);
		}
		else if(manipStick->GetRawButton(11)){
			robotElevator->SetElevatorTarget(ELEVATOR_BOTTOM_HEIGHT);
		}
		robotElevator->MoveElevator(-.8*manipStick->GetRawAxis(1));//updates elevator positions based on targets and joysticks

		if(manipStick->GetRawButton(8) || manipStick->GetRawButton(10) || manipStick->GetRawButton(12)){
			robotElevator->SetJoystickControl();
		}
		//runs intake


		if(manipStick->GetRawButton(2) || mainDriverStick->GetRawButton(1)){
			robotIntake->OuttakeCubes();
			wasIntaking = false;
		}
		else if(manipStick->GetRawButton(1) || mainDriverStick->GetRawButton(7)){
			robotIntake->IntakeCubes();
			wasIntaking = true;
		}
		else if(wasIntaking){
			robotIntake->HoldIntake();
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

		//runs climber
		if(manipStick->GetRawButton(12)){
			robotClimber->SpoolClimber(true);
		}
		else{
		robotClimber->SpoolClimber(false);
		}



		//Prints some relevant stuff
	}
	void TestInit() {
			printf( "TestInit...\n" );
		}

	void TestPeriodic() {
		printf("TestPeriodic \n");
		if(mainDriverStick->GetRawButton(11)) {
			printf( "Recalculating Motion Trajectories...\n" );

			robotDrive->Load_Waypoints();	// Load all the data into the waypoint structures.

			// Step thru and calculate each path.  The result is stored as a
			// binary/CSV file on the RoboRIO file-system.
			for ( int idx=0 ; idx < NUM_PATHS ; idx++ ) {
				printf( "Calculating PathFinder Path: %d\n", idx );
				robotDrive->FindPath( idx );
			}
			printf( "Info: PathFinder Done\n" );
		}
	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();

	frc::SendableChooser<std::string> auton_chooser;
	const std::string DriveStraight= "DriveStraight";
	const std::string Center1Cube = "Center1Cube";
	const std::string Left1Cube = "Left1Cube";
	const std::string Right1Cube = "Right1Cube";

	frc::SendableChooser<std::string> priority_chooser;
	const std::string Switch = "Switch";
	const std::string Scale = "Scale";

	std::string m_autoSelected;
	std::string m_prioritySelected;

};

START_ROBOT_CLASS(Robot)
