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
#include <Auton/Auton.h>

#define ELEVATOR_BOTTOM_HEIGHT -1200
#define ELEVATOR_SWITCH_HEIGHT 4500	//untested
#define ELEVATOR_SCALE_HEIGHT 14000 //untested

#define TURN_FACTOR 0.5
#define DRIVE_SPEED_FACTOR 1.0
#define RIGHT_DRIVE_CORRECTION 0.96
#define DRIVE_SCALE 1.7
#define TURN_SCALE 1.1

//TODO Find all the following distances/angles

//Auton Drive Straight Distance
#define CL_ZEROA 130.0

//Auton Center Distances
#define C1_ONEC 32.0
#define C2_ONEC 45.0
#define C3_ONEC 70.0

//Auton Center Angles
#define T1_ONEC 90.0
#define T2_ONEC 90.0

//Auton Sides Switch Distances
#define C1_SWITCH_ONES 144.0
#define C2_SWITCH_ONES 12.0

//Auton Sides Switch Angle
#define T1_SWITCH_ONES 90.0

//Auton Sides Scale Distances
#define C1_SCALE_ONES 240.0
#define C2_SCALE_ONES 12.0

//Auton Sides Scale Angle
#define T1_SCALE_ONES 90.0

//Auton Sides Far Cube Distances
#define C1_ZEROS 180.0
#define C2_ZEROS 72.0
#define C3_ZEROS 24.0

//Auton Sides Far Cube Angle
#define T1_ZEROS 90.0
#define T2_ZEROS 90.0

class Robot : public frc::TimedRobot {
public:

	//Initialize the subsystems

	Intake *robotIntake;
	Drive *robotDrive;
	double driveRevFactor = 1.0;
	bool wasButtonPressed = false;
    Elevator *robotElevator;

	//init sensors
	DigitalInput *elevatorBottomSwitch;
	bool wasSwitchPressed;

	double startYaw;
	int firstPriority;
	int secondPriority;
	double matchTime;

	Timer *initTimer = new Timer();

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;


	/* Auton Naming Rules:
	 * c = constant speed forward driving
	 * t = turn
	 * e = set elevator target
	 * m = move elevator
	 * o = outtake
	 * d = deploy
	 * i = intake
	 * s = stow
	 * fin = fin_ZeroA
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

	//AutonDriveStraight
	enum ZeroA {c1_ZeroA, fin_ZeroA};
	ZeroA statusZeroA = c1_ZeroA;

	//AutonCenter
	enum OneC {c1_OneC,t1_OneC,c2_OneC,t2_OneC,c3_OneC,e1_OneC,m1_OneC,d1_OneC,o1_OneC,s1_OneC,e2_OneC,m2_OneC,fin_OneC};
	OneC statusOneC = c1_OneC;

	//AutonSidesSwitchOrScale
	enum OneS {c1_OneS,t1_OneS,c2_OneS,e1_OneS,m1_OneS,d1_OneS,o1_OneS,s1_OneS,e2_OneS,m2_OneS,fin_OneS};
	OneS statusOneS = c1_OneS;

	//AutonSidesBothAreOnOppositeSide
	enum ZeroS {c1_ZeroS,t1_ZeroS,c2_ZeroS,t2_ZeroS,c3_ZeroS,e1_ZeroS,m1_ZeroS,d1_ZeroS,o1_ZeroS,s1_ZeroS,e2_ZeroS,m2_ZeroS,fin_ZeroS};
	ZeroS statusZeroS = c1_ZeroS;


	void RobotInit() {
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
		std::thread visionThread(VisionThread);
		visionThread.detach();

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


		startYaw = robotDrive->GetYaw();

		//Sets first case for auton enums
		statusZeroA = c1_ZeroA;
		statusOneC = c1_OneC;
		statusOneS = c1_OneS;
		statusZeroS = c1_ZeroS;

		//temporarily overrides DS to pick our own auton
			//m_autoSelected = "DriveStraight";
			//m_autoSelected = "Center1Cube";
			//m_autoSelected = "Right1Cube";
			//m_autoSelected = "Left1Cube"

		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();


		if (m_autoSelected == "Center1Cube") {
			robotDrive->ResetEncoders();
			robotDrive->ResetYaw();
		}
		if (m_autoSelected == "Left1Cube") {
			robotDrive->ResetEncoders();
			robotDrive->ResetYaw();
				}
		if (m_autoSelected == "Right1Cube") {
			robotDrive->ResetEncoders();
			robotDrive->ResetYaw();
		}
		else {
			robotDrive->ResetEncoders();
			robotDrive->ResetYaw();
		}
	}

	void AutonomousPeriodic() {
			std::string gameData;
			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
			//printf("Encoder dist: %f\n", robotDrive->GetAverageEncoderDistance());
			//printf("Selected Auton %s\n",m_autoSelected.c_str());

			matchTime = DriverStation::GetInstance().GetMatchTime();
			frc::SmartDashboard::PutNumber("Match Time", matchTime);

			//printf("Current Yaw %f \n", robotDrive->GetYaw());
			if (m_autoSelected == Center1Cube) {

			//Following Code Runs if Center1Cube is the selected auton

				if(gameData.length() > 0){

				//Following Code Runs if the Robot is Recieving the Orientation of the Switches and Scale

					if(gameData[0] == 'L')  {

					//Following Code Runs if the Alliance Switch is on the Left

						switch(statusOneC){

						//drive
						case c1_OneC:
							if(AutonDriveStraight(C1_ONEC, robotDrive,0.0)){
								startYaw = robotDrive->GetYaw(); statusOneC = t1_OneC;
							}
							break;

						//turn
						case t1_OneC:
							if(AutonTurnLeft(T1_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = c2_OneC;
							}
							break;

						//drive
						case c2_OneC:
							if(AutonDriveStraight(C2_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = t2_OneC;
							}
							break;

						//turn
						case t2_OneC:
							printf("turn2 \n");
							if(AutonTurnRight(T2_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = c3_OneC;
							}
							break;

						//drive
						case c3_OneC:
							if(AutonDriveStraight(C3_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = e1_OneC;
							}
							break;

						//set elevator target to switch
						case e1_OneC:
							if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneC = m1_OneC;
							}
							break;

						//move elevator to elevator targer
						case m1_OneC:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusOneC = d1_OneC;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_OneC:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneC = o1_OneC;
							}
							break;

						//outtake
						case o1_OneC:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneC = s1_OneC;
							}
							break;

						//stow intake
						case s1_OneC:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusOneC = e2_OneC;
							}
							break;

						//set elevator target to bottom position
						case e2_OneC:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneC = m2_OneC;
							}
							break;

						//move elevator to target height
						case m2_OneC:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneC = fin_OneC;
							}
							break;


						//end auton
						case fin_OneC:
						//do nothing
							break;
						default:
						//do nothing
							break;
						}
					}
					else {

					//Following Code Runs if the Alliance Switch is on the Right

						switch(statusOneC){

						//drive
						case c1_OneC:
							if(AutonDriveStraight(C1_ONEC, robotDrive, 0.0)){
								startYaw = robotDrive->GetYaw(); statusOneC = t1_OneC;
							}
							break;

						//turn
						case t1_OneC:
							if(AutonTurnRight(T1_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = c2_OneC;
							}
							break;

						//drive
						case c2_OneC:
							if(AutonDriveStraight(C2_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = t2_OneC;
							}
							break;

						//turn
						case t2_OneC:
							if(AutonTurnLeft(T2_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = c3_OneC;
							}
							break;

						//drive
						case c3_OneC:
							if(AutonDriveStraight(C3_ONEC, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneC = e1_OneC;
							}
							break;

						//set elevator target to switch position
						case e1_OneC:
							if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneC = m1_OneC;
							}
							break;

						//move elevator to elevator target
						case m1_OneC:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusOneC = d1_OneC;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_OneC:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneC = o1_OneC;
							}
							break;

						//outtake
						case o1_OneC:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneC = s1_OneC;
							}
							break;

						//stow intake
						case s1_OneC:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusOneC = e2_OneC;
							}
							break;

						//set elevator target to bottom position
						case e2_OneC:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneC = m2_OneC;
							}
							break;

						//move elevator to elevator target
						case m2_OneC:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneC = fin_OneC;
							}
							break;

						//end auton
						case fin_OneC:
							//do nothing
							break;
						default:
							//do nothing
							break;
						}
					}
				}
			}
			else if (m_autoSelected == Left1Cube) {

			//Following Code Runs if Left1Cube is the selected auton

				if(gameData.length() > 0){

				//Following Code Runs if the Robot is Recieving the Orientation of the Switches and Scale

					if(gameData[firstPriority] == 'L'){

					//Following Code Runs if the Alliance Switch is on the Left

						switch(statusOneS){

						//drive
						case c1_OneS:
							if(AutonDriveStraight(C1_SWITCH_ONES, robotDrive, 0.0)){
								startYaw = robotDrive->GetYaw(); statusOneS = t1_OneS;
							}
							break;

						//turn
						case t1_OneS:
							if(AutonTurnRight(T1_SWITCH_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = c2_OneS;
							}
							break;

						//drive
						case c2_OneS:
							if(AutonDriveStraight(C2_SWITCH_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = e1_OneS;
							}
							break;

						//set elevator target to switch position
						case e1_OneS:
							if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m1_OneS;
							}
							break;

						//move elevator to elevator target
						case m1_OneS:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusOneS = d1_OneS;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_OneS:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = o1_OneS;
							}
							break;

						//outtake
						case o1_OneS:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = s1_OneS;
							}
							break;

						//stow intake
						case s1_OneS:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusOneS = e2_OneS;
							}
							break;

						//set elevator target to bottom position
						case e2_OneS:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m2_OneS;
							}
							break;

						//move elevator to elevator target
						case m2_OneS:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = fin_OneS;
							}
							break;

						//end auton
						case fin_OneS:
							//do nothing
							break;
						default:
							//do nothing
							break;
						}
					}
					else {

					//Following Code Runs if the Alliance Switch is on the Right

						if(gameData[secondPriority] == 'L') {

						//Following Code Runs if the Alliance Scale is on the Left

						switch(statusOneS){

						//drive
						case c1_OneS:
							if(AutonDriveStraight(C1_SCALE_ONES, robotDrive, 0.0)){
								startYaw = robotDrive->GetYaw(); statusOneS = t1_OneS;
							}
							break;

						//turn
						case t1_OneS:
							if(AutonTurnRight(T1_SCALE_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = c2_OneS;
							}
							break;

						//drive
						case c2_OneS:
							if(AutonDriveStraight(C2_SCALE_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = e1_OneS;
							}
							break;

						//set elevator target to Scale position
						case e1_OneS:
							if(AutonSetHeight(ELEVATOR_SCALE_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m1_OneS;
							}
							break;

						//move elevator to elevator target
						case m1_OneS:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusOneS = d1_OneS;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_OneS:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = o1_OneS;
							}
							break;

						//outtake
						case o1_OneS:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = s1_OneS;
							}
							break;

						//stow intake
						case s1_OneS:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusOneS = e2_OneS;
							}
							break;

						//set elevator target to bottom position
						case e2_OneS:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m2_OneS;
							}
							break;

						//move elevator to elevator target
						case m2_OneS:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = fin_OneS;
							}
							break;

						//end auton
						case fin_OneS:
							//do nothing
							break;
						default:
							//do nothing
							break;
						}
						}
						else {

						//Following Code Runs if the Alliance Scale is on the Right

						switch(statusZeroS){

						//drive
						case c1_ZeroS:
							if(AutonDriveStraight(C1_ZEROS, robotDrive, 0.0)){
								startYaw = robotDrive->GetYaw(); statusZeroS = t1_ZeroS;
							}
							break;

						//turn
						case t1_ZeroS:
							if(AutonTurnRight(T1_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = c2_ZeroS;
							}
							break;

						//drive
						case c2_ZeroS:
							if(AutonDriveStraight(C2_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = t2_ZeroS;
							}
							break;

						//turn
						case t2_ZeroS:
							if(AutonTurnLeft(T2_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = c3_ZeroS;
							}
							break;

						//drive
						case c3_ZeroS:
							if(AutonDriveStraight(C3_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = e1_ZeroS;
							}
							break;
							//set elevator target to Scale position
						case e1_ZeroS:
							if(AutonSetHeight(ELEVATOR_SCALE_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusZeroS = m1_ZeroS;
							}
							break;

						//move elevator to elevator target
						case m1_ZeroS:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusZeroS = d1_ZeroS;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_ZeroS:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusZeroS = o1_ZeroS;
							}
							break;

						//outtake
						case o1_ZeroS:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusZeroS = s1_ZeroS;
							}
							break;

						//stow intake
						case s1_ZeroS:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusZeroS = e2_ZeroS;
							}
							break;

						//set elevator target to bottom position
						case e2_ZeroS:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusZeroS = m2_ZeroS;
							}
							break;

						//move elevator to elevator target
						case m2_ZeroS:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusZeroS = fin_ZeroS;
							}
							break;

						//end auton
						case fin_ZeroS:
							//do nothing
							break;
						default:
							//do nothing
							break;
						}
						}
					}
				}
			}
			else if (m_autoSelected == Right1Cube) {

			//Following Code Runs if Right1Cube is the selected auton

				if(gameData.length() > 0){

				//Following Code Runs if the Robot is Recieving the Orientation of the Switches and Scale

					if(gameData[firstPriority] == 'R'){

					//Following Code Runs if the Alliance Switch is on the Right

						switch(statusOneS){

						//drive
						case c1_OneS:
							if(AutonDriveStraight(C1_SWITCH_ONES, robotDrive, 0.0)){
								startYaw = robotDrive->GetYaw(); statusOneS = t1_OneS;
							}
							break;

						//turn
						case t1_OneS:
							if(AutonTurnLeft(T1_SWITCH_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = c2_OneS;
							}
							break;

						//drive
						case c2_OneS:
							if(AutonDriveStraight(C2_SWITCH_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = e1_OneS;
							}
							break;

						//set elevator target to switch position
						case e1_OneS:
							if(AutonSetHeight(ELEVATOR_SWITCH_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m1_OneS;
							}
							break;

						//move elevator to elevator target
						case m1_OneS:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusOneS = d1_OneS;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_OneS:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = o1_OneS;
							}
							break;

						//outtake
						case o1_OneS:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = s1_OneS;
							}
							break;

						//stow intake
						case s1_OneS:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusOneS = e2_OneS;
							}
							break;

						//set elevator target to bottom position
						case e2_OneS:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m2_OneS;
							}
							break;

						//move elevator to elevator target
						case m2_OneS:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = fin_OneS;
							}
							break;

						//end auton
						case fin_OneS:
							//do nothing
							break;
						default:
							//do nothing
							break;
						}
					}
					else {

					//Following Code Runs if the Alliance Switch is on the Left

						if(gameData[secondPriority] == 'R') {

						//Following Code Runs if the Alliance Scale is on the Right

						switch(statusOneS){

						//drive
						case c1_OneS:
							if(AutonDriveStraight(C1_SCALE_ONES, robotDrive, 0.0)){
								startYaw = robotDrive->GetYaw(); statusOneS = t1_OneS;
							}
							break;

						//turn
						case t1_OneS:
							if(AutonTurnLeft(T1_SCALE_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = c2_OneS;
							}
							break;

						//drive
						case c2_OneS:
							if(AutonDriveStraight(C2_SCALE_ONES, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusOneS = e1_OneS;
							}
							break;

						//set elevator target to Scale position
						case e1_OneS:
							if(AutonSetHeight(ELEVATOR_SCALE_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m1_OneS;
							}
							break;

						//move elevator to elevator target
						case m1_OneS:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusOneS = d1_OneS;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_OneS:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = o1_OneS;
							}
							break;

						//outtake
						case o1_OneS:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusOneS = s1_OneS;
							}
							break;

						//stow intake
						case s1_OneS:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusOneS = e2_OneS;
							}
							break;

						//set elevator target to bottom position
						case e2_OneS:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = m2_OneS;
							}
							break;

						//move elevator to elevator target
						case m2_OneS:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusOneS = fin_OneS;
							}
							break;

						//end auton
						case fin_OneS:
							//do nothing
							break;
						default:
							//do nothing
							break;
						}
						}
						else {

						//Following Code Runs if the Alliance Scale is on the Left

						switch(statusZeroS){

						//drive
						case c1_ZeroS:
							if(AutonDriveStraight(C1_ZEROS, robotDrive, 0.0)){
								startYaw = robotDrive->GetYaw(); statusZeroS = t1_ZeroS;
							}
							break;

						//turn
						case t1_ZeroS:
							if(AutonTurnLeft(T1_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = c2_ZeroS;
							}
							break;

						//drive
						case c2_ZeroS:
							if(AutonDriveStraight(C2_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = t2_ZeroS;
								}
							break;

						//turn
						case t2_ZeroS:
							if(AutonTurnRight(T2_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = c3_ZeroS;
							}
							break;

						//drive
						case c3_ZeroS:
							if(AutonDriveStraight(C3_ZEROS, robotDrive, startYaw)){
								startYaw = robotDrive->GetYaw(); statusZeroS = e1_ZeroS;
							}
							break;
							//set elevator target to Scale position
						case e1_ZeroS:
							if(AutonSetHeight(ELEVATOR_SCALE_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusZeroS = m1_ZeroS;
							}
							break;

						//move elevator to elevator target
						case m1_ZeroS:
							if(AutonMoveToHeight(robotElevator)){
								robotElevator->SetToOutput(0.1);
								startYaw = robotDrive->GetYaw(); statusZeroS = d1_ZeroS;
								//Initiates timer for deploying
								initTimer->Reset();
								initTimer->Start();
							}
							break;

						//deploy intake
						case d1_ZeroS:
							if(AutonDeployIntake(robotIntake) && (initTimer->Get() > 2.0)){
								initTimer->Stop();

								//Initiates timer for outtake
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusZeroS = o1_ZeroS;
							}
							break;

						//outtake
						case o1_ZeroS:
							printf("Outtake Timer: %f\n", initTimer->Get());
							if(AutonOuttake(robotIntake) && (initTimer->Get() > 0.5)){
								AutonStopIntake(robotIntake);
								initTimer->Stop();
								//Initiates timer for stowage
								initTimer->Reset();
								initTimer->Start();
								startYaw = robotDrive->GetYaw(); statusZeroS = s1_ZeroS;
							}
							break;

						//stow intake
						case s1_ZeroS:
							if(AutonStowIntake(robotIntake) && (initTimer->Get() > 1.5)){
								startYaw = robotDrive->GetYaw(); statusZeroS = e2_ZeroS;
							}
							break;

						//set elevator target to bottom position
						case e2_ZeroS:
							if(AutonSetHeight(ELEVATOR_BOTTOM_HEIGHT,robotElevator)){
								startYaw = robotDrive->GetYaw(); statusZeroS = m2_ZeroS;
							}
							break;

						//move elevator to elevator target
						case m2_ZeroS:
							if(AutonMoveToHeight(robotElevator)){
								startYaw = robotDrive->GetYaw(); statusZeroS = fin_ZeroS;
							}
							break;

						//end auton
						case fin_ZeroS:
							//do nothing
							break;
						default:
							//do nothing
							break;
						}
						}
					}
				}
			}
			else {

			//Following Code Runs if DriveStraight is the selected auton (default)

				switch(statusZeroA){

				//drive
				case c1_ZeroA:
					//if(AutonDriveStraight(CL_ZEROA, robotDrive, 0.0)){
					if(AutonDriveStraight(90, robotDrive, 0.0)){
						startYaw = robotDrive->GetYaw(); statusZeroA = fin_ZeroA;
					}
					break;

				//end auton
				case fin_ZeroA:
					robotDrive->TankDrive(0.0,0.0);
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
		robotDrive->ResetYaw();
		robotDrive->SetCoastMode();
	}

	void TeleopPeriodic() {
		matchTime = DriverStation::GetInstance().GetMatchTime();
		frc::SmartDashboard::PutNumber("Match Time", matchTime);

		//drives robot according to joystick inputs
		double speedVal  = robotDrive->InputScale(DRIVE_SPEED_FACTOR * mainDriverStick->GetRawAxis(1), DRIVE_SCALE);
		double turnVal = robotDrive->InputScale(TURN_FACTOR * mainDriverStick->GetRawAxis(4), TURN_SCALE);

		//updates drive direction
		if(mainDriverStick->GetRawButton(6) && !wasButtonPressed){ //if this button is pressed for the first time:
			wasButtonPressed = true;
			driveRevFactor *= -1.0; //reverse drive direction
		}
		else if(!mainDriverStick->GetRawButton(6)){
			wasButtonPressed = false;
		}

		robotDrive->ArcadeDrive( driveRevFactor * speedVal, turnVal);

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

	static void VisionThread() {
			// Get the Axis camera from CameraServer
			cs::AxisCamera camera =
					CameraServer::GetInstance()->AddAxisCamera(
							"axis-camera.local");
			// Set the resolution
			camera.SetResolution(640, 480);

			// Get a CvSink. This will capture Mats from the Camera
			cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			cs::CvSource outputStream =
					CameraServer::GetInstance()->PutVideo(
							"Rectangle", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			cv::Mat mat;

			while (true) {
				// Tell the CvSink to grab a frame from the camera and
				// put it
				// in the source mat.  If there is an error notify the
				// output.
				if (cvSink.GrabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.NotifyError(cvSink.GetError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
						cv::Scalar(255, 255, 255), 5);
				// Give the output stream a new image to display
				outputStream.PutFrame(mat);
			}
		}
};

START_ROBOT_CLASS(Robot)
