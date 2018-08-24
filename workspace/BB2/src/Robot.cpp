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
#include <Auton/SequentialCommand.h>
#include <Auton/ConcurrentCommand.h>
#include <Auton/AutonIntake.h>
#include <Auton/AutonDriveStraight.h>
#include <Auton/AutonMoveElevatorToHeight.h>
#include <Auton/AutonOuttake.h>
#include <Auton/AutonDeployIntake.h>
#include <Auton/AutonStowIntake.h>
#include <Auton/AutonTurnLeft.h>
#include <Auton/AutonTurnRight.h>
#include <Auton/AutonTurnLeft1.h>
#include <Auton/AutonTurnRight1.h>
#include <Auton/MotionMagicStraight.h>
#include <Auton/MotionMagicTurn.h>
#include <Auton/AutonOpenClaw.h>
#include <Auton/AutonCloseClaw.h>
#include <Auton/AutonStowIntake1.h>

#define ELEVATOR_BOTTOM_HEIGHT -600
#define ELEVATOR_SWITCH_HEIGHT 5500	//TODO untested
#define ELEVATOR_SCALE_HEIGHT 13600 //untested

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
#define CL_ZEROA 130.0 //TODO fix measurement

//Auton Center Distances
#define C1_ONEC 30.42
#define C2_ONEC 46.30
#define C3_ONEC 61.0

//Auton Center Angles
#define T1_ONEC 90.0
#define T2_ONEC 90.0

//Auton Sides Switch Distances
#define C1_SWITCH_ONES 145.16
#define C2_SWITCH_ONES 18.0

//Auton Sides Switch Angle
#define T1_SWITCH_ONES 90.0

//Auton Sides Scale Distances //TODO Change to 235.00
#define C1_SCALE_ONES 282.1
#define C2_SCALE_ONES 0.0 //TODO TUNE THIS
#define C3_SCALE_ONES 18.0

//Auton Sides Scale Angle
#define T1_SCALE_ONES 80.0

//Auton Sides Far Cube Scale Distances
#define C1_ZEROS 210.9
#define C2_ZEROS 50.0 //Change to 214.32
#define C3_ZEROS 0.0 // Change to 34.04

//Auton Sides Far Cube Scale Angle
#define T1_ZEROS 92.0
#define T2_ZEROS 95.0

//Auton Close Side 2 Cube Distances
#define C1_TWOS 253.47
#define C2_TWOS 13.7
#define C3_TWOS 62.19
#define C4_TWOS 9.12
#define C5_TWOS 66.15

//Auton Close Side 2 Cube Angles
#define T1_TWOS 45.0
#define T2_TWOS 115.71
#define T3_TWOS 19.29
#define T4_TWOS 173.61

//Auton Center 2 Cube Distances
#define C1_TWOC 12.79
#define C2_TWOC 82.96
#define C3_TWOC 54.2
#define C4_TWOC 9.85

#define T1_TWOC 21.42
#define T2_TWOC 13.86
#define T3_TWOC 35.27

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

	double startYaw;
	int firstPriority;
	int secondPriority;
	double matchTime;

	SequentialCommand* mainAutoCommand = NULL;
	SequentialCommand* elevatorSwitchCommand;
	SequentialCommand* elevatorScaleCommand;
	ConcurrentCommand* intakeCommand;

	//Init joysticks
	Joystick *mainDriverStick, *secondaryDriverStick, *manipStick;


	void RobotInit() {
		//populates auto chooser on dashboard
		auton_chooser.AddDefault(DriveStraight, DriveStraight);
		auton_chooser.AddObject(Center1Cube, Center1Cube);
		auton_chooser.AddObject(Left1Cube, Left1Cube);
		auton_chooser.AddObject(Right1Cube, Right1Cube);
		auton_chooser.AddObject(Left2Cube, Left2Cube);
		auton_chooser.AddObject(Right2Cube, Right2Cube);
		auton_chooser.AddObject(RightSwitchOnly, RightSwitchOnly);
		auton_chooser.AddObject(LeftSwitchOnly,LeftSwitchOnly);
		auton_chooser.AddObject(Center2Cube,Center2Cube);

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
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();


		//initialize subsystems
		robotDrive = new Drive(2,1,4,3,0); 			//drive uses Talons 1,2,3,4 and pigeonIMU port 0
		robotElevator = new Elevator(5); 		//elevator uses Talon 5 and DIOs 0 and 1
		robotIntake = new Intake(0,1,0,1,2);		//Intake uses PWM 0 and 1, and PCM ports 0, 1, and 2
		robotClimber = new Climber(9);


		//initialize sensors
		elevatorBottomSwitch = new DigitalInput(0);
		wasSwitchPressed = 0;

		//reset sensors
		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();
		robotDrive->ResetFusedHeading();

		robotElevator->SetJoystickControl();
		robotDrive->FollowMode();
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
				new AutonStowIntake1(robotIntake, 90),
				new AutonOuttake(robotIntake, 1.5),
				new AutonStowIntake1(robotIntake, 180),
				new AutonMoveElevatorToHeight(robotElevator, ELEVATOR_BOTTOM_HEIGHT));

		intakeCommand = AUTO_CONCURRENT(
				new AutonOpenClaw(robotIntake),
				new AutonIntake(robotIntake, 1.0));


		std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		if(gameData.length()>0){//if the auton data exists
			if(m_autoSelected == Center1Cube){//if center auton is selected

				if(gameData[0] == 'L'){//creates left switch from center auton
					mainAutoCommand = AUTO_SEQUENTIAL(
							new MotionMagicStraight(robotDrive, C1_ONEC),
							new AutonTurnLeft1(robotDrive, T1_ONEC),
							new MotionMagicStraight(robotDrive, C2_ONEC),
							new AutonTurnRight1(robotDrive, T2_ONEC),
							new MotionMagicStraight(robotDrive, C3_ONEC),
							elevatorSwitchCommand);
				}
				else{//otherwise creates right switch from center auton
					mainAutoCommand = AUTO_SEQUENTIAL(
							new MotionMagicStraight(robotDrive, C1_ONEC),
							new AutonTurnRight1(robotDrive, T1_ONEC),
							new MotionMagicStraight(robotDrive, C2_ONEC),
							new AutonTurnLeft1(robotDrive, T2_ONEC),
							new MotionMagicStraight(robotDrive, C3_ONEC),
							elevatorSwitchCommand);
				}
			}
			else if(m_autoSelected == Center2Cube){//if center auton is selected

				if(gameData[0] == 'L'){//creates left switch from center auton
					mainAutoCommand = AUTO_SEQUENTIAL(
							new MotionMagicStraight(robotDrive, C1_TWOC),
							new AutonTurnLeft1(robotDrive, T1_TWOC),
							new MotionMagicStraight(robotDrive, C2_TWOC),
							new AutonStowIntake1(robotIntake,90),
							new AutonOuttake(robotIntake,1.0),
							new AutonTurnLeft1(robotDrive, T2_TWOC),
							new MotionMagicStraight(robotDrive, -1.0 * C3_TWOC),
							new AutonTurnRight1(robotDrive, T3_TWOC),
							new AutonDeployIntake(robotIntake),
							new AutonOpenClaw(robotIntake),
							new MotionMagicStraight(robotDrive,C4_TWOC),
							new AutonCloseClaw(robotIntake),
							new AutonIntake(robotIntake, 0.7),
							new MotionMagicStraight(robotDrive, -1.0 * C4_TWOC),
							new AutonTurnLeft1(robotDrive,T3_TWOC),
							new AutonStowIntake1(robotIntake,90),
							new MotionMagicStraight(robotDrive,C3_TWOC),
							new AutonOuttake(robotIntake,1.0));
				}
				else{//otherwise creates right switch from center auton
					mainAutoCommand = AUTO_SEQUENTIAL(
							new MotionMagicStraight(robotDrive, C1_TWOC),
							new AutonTurnRight1(robotDrive, T1_TWOC),
							new MotionMagicStraight(robotDrive, C2_TWOC),
							new AutonStowIntake1(robotIntake,90),
							new AutonOuttake(robotIntake,1.0),
							new AutonTurnRight1(robotDrive, T2_TWOC),
							new MotionMagicStraight(robotDrive, -1.0 * C3_TWOC),
							new AutonTurnLeft1(robotDrive, T3_TWOC),
							new AutonDeployIntake(robotIntake),
							new AutonOpenClaw(robotIntake),
							new MotionMagicStraight(robotDrive,C4_TWOC),
							new AutonCloseClaw(robotIntake),
							new AutonIntake(robotIntake, 0.7),
							new MotionMagicStraight(robotDrive, -1.0 * C4_TWOC),
							new AutonTurnRight1(robotDrive,T3_TWOC),
							new AutonStowIntake1(robotIntake,90),
							new MotionMagicStraight(robotDrive,C3_TWOC),
							new AutonOuttake(robotIntake,1.0));
				}
			}

			else if(m_autoSelected == Left1Cube){//if cube auton from left is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'L'){//if scale is on left go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnRight1(robotDrive, T1_SCALE_ONES),
								new MotionMagicStraight(robotDrive, C2_SCALE_ONES),
								new MotionMagicStraight(robotDrive, -1.0 * C3_SCALE_ONES),
								elevatorScaleCommand);
					}
					else{
						if(gameData[0] == 'L'){//if switch is on left go for that
							mainAutoCommand = AUTO_SEQUENTIAL(
									new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
									new AutonTurnRight1(robotDrive, T1_SWITCH_ONES),
									new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
									elevatorSwitchCommand);
						}
						else{//otherwise go for right scale but don't drop cube
							mainAutoCommand = AUTO_SEQUENTIAL(
									new MotionMagicStraight(robotDrive, C1_ZEROS),
									new AutonTurnRight1(robotDrive, T1_ZEROS),
									new MotionMagicStraight(robotDrive, C2_ZEROS)
									/*new AutonTurnLeft1(robotDrive, T2_ZEROS),
									new MotionMagicStraight(robotDrive,C3_ZEROS)*/);
						}
					}
				}
				else{//if priority is switch
					if(gameData[0] == 'L'){//if switch is on left go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnRight1(robotDrive, T1_SWITCH_ONES),
								new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for scale auton
						if(gameData[1] == 'L'){//if scale is on left go for that
							mainAutoCommand = AUTO_SEQUENTIAL(
									new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
									new AutonTurnRight1(robotDrive, T1_SCALE_ONES),
									new MotionMagicStraight(robotDrive, C2_SCALE_ONES),
									new MotionMagicStraight(robotDrive, -1.0 * C3_SCALE_ONES),
									elevatorScaleCommand,
									new MotionMagicStraight(robotDrive, -1.0 * C3_SCALE_ONES));
						}
						else{//otherwise go for right scale but don't drop cube
							mainAutoCommand = AUTO_SEQUENTIAL(
									new MotionMagicStraight(robotDrive, C1_ZEROS),
									new AutonTurnRight1(robotDrive, T1_ZEROS),
									new MotionMagicStraight(robotDrive, C2_ZEROS)
									/*new AutonTurnLeft1(robotDrive, T2_ZEROS),
									new MotionMagicStraight(robotDrive,C3_ZEROS)*/);
						}
					}
				}
			}
			else if(m_autoSelected == LeftSwitchOnly){//if cube auton from left is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'L'){//if scale is on left go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnRight1(robotDrive, T1_SCALE_ONES),
								new MotionMagicStraight(robotDrive, C2_SCALE_ONES));
					}
					else{//otherwise go for right scale but don't drop cube
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnRight1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS),
								new AutonTurnLeft1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS));

					}

				}
				else{//if priority is switch
					if(gameData[0] == 'L'){//if switch is on left go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnRight1(robotDrive, T1_SWITCH_ONES),
								new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for scale auton
						if(gameData[1] == 'L'){//if scale is on left go for that
							mainAutoCommand = AUTO_SEQUENTIAL(
									new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
									new AutonTurnRight1(robotDrive, T1_SCALE_ONES),
									new MotionMagicStraight(robotDrive, C2_SCALE_ONES));
						}
						else{//otherwise go for right scale but don't drop cube
							mainAutoCommand = AUTO_SEQUENTIAL(
									new MotionMagicStraight(robotDrive, C1_ZEROS),
									new AutonTurnRight1(robotDrive, T1_ZEROS),
									new MotionMagicStraight(robotDrive, C2_ZEROS),
									new AutonTurnLeft1(robotDrive, T2_ZEROS),
									new MotionMagicStraight(robotDrive,C3_ZEROS));
						}
					}
				}
			}
			else if(m_autoSelected == RightSwitchOnly){//if cube auton from right is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'R'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnLeft1(robotDrive, T1_SCALE_ONES),
								new MotionMagicStraight(robotDrive, C2_SCALE_ONES));
					}
					else{//otherwise go for opposite scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS),
								new AutonTurnRight1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS));
					}

				}
				else{//if priority is switch
					if(gameData[0] == 'R'){//if switch is on right side, go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnLeft1(robotDrive, T1_SWITCH_ONES),
								new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for a scale auton
						if(gameData[1] == 'R'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnLeft1(robotDrive, T1_SCALE_ONES),
								new MotionMagicStraight(robotDrive, C2_SCALE_ONES));
					}
						else{//otherwise go for opposite scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS),
								new AutonTurnRight1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS));
						}
					}
				}
			}
			else if(m_autoSelected == Right1Cube){//if cube auton from right is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'R'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnLeft1(robotDrive, T1_SCALE_ONES),
								new MotionMagicStraight(robotDrive, C2_SCALE_ONES),
								new MotionMagicStraight(robotDrive, -1.0 * C3_SCALE_ONES),
								elevatorScaleCommand);
					}
					else{
						if(gameData[0] == 'R'){//if switch is on right side, go for that
							mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnLeft1(robotDrive, T1_SWITCH_ONES),
								new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
						}
						else{//otherwise go for a far scale auton
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS)
								/*new AutonTurnRight1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS)*/);
						}

					}
				}
				else{//if priority is switch
					if(gameData[0] == 'R'){//if switch is on right side, go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnLeft1(robotDrive, T1_SWITCH_ONES),
								new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for a scale auton
						if(gameData[1] == 'R'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SCALE_ONES),
								new AutonTurnLeft1(robotDrive, T1_SCALE_ONES),
								new MotionMagicStraight(robotDrive, C2_SCALE_ONES),
								new MotionMagicStraight(robotDrive, -1.0 * C3_SCALE_ONES),
								elevatorScaleCommand,
								new MotionMagicStraight(robotDrive, -1.0 * C3_SCALE_ONES));
					}
					else{//otherwise go for opposite scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS)
								/*new AutonTurnRight1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS)*/);
					}
					}
				}
			}
			else if(m_autoSelected == Left2Cube){//if cube auton from right is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'L'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_TWOS),
								new AutonTurnRight1(robotDrive, T1_TWOS),
								new MotionMagicStraight(robotDrive, C2_TWOS),
								elevatorScaleCommand,
								new AutonDeployIntake(robotIntake),
								new AutonOpenClaw(robotIntake),
								new AutonTurnRight1(robotDrive,T2_TWOS),
								new MotionMagicStraight(robotDrive,C3_TWOS),
								new AutonTurnRight1(robotDrive,T3_TWOS),
								new MotionMagicStraight(robotDrive,C4_TWOS),
								intakeCommand,
								new AutonStowIntake(robotIntake),
								new AutonTurnRight1(robotDrive, T4_TWOS),
								new MotionMagicStraight(robotDrive,C5_TWOS),
								elevatorScaleCommand);
					}
					else{//otherwise go for opposite scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS),
								new AutonTurnRight1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS),
								elevatorScaleCommand);
					}

				}
				else{//if priority is switch
					if(gameData[0] == 'L'){//if switch is on right side, go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnRight1(robotDrive, T1_SWITCH_ONES),
								new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for a scale auton
						if(gameData[1] == 'L'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_TWOS),
								new AutonTurnLeft1(robotDrive, T1_TWOS),
								new MotionMagicStraight(robotDrive, C2_TWOS),
								elevatorScaleCommand,
								new AutonDeployIntake(robotIntake),//add OpenClaw
								new AutonTurnLeft1(robotDrive,T2_TWOS),
								new MotionMagicStraight(robotDrive,C3_TWOS),
								new AutonTurnLeft1(robotDrive,T3_TWOS),
								new MotionMagicStraight(robotDrive,C4_TWOS), //closeClaw
								intakeCommand,
								new AutonStowIntake(robotIntake),
								new AutonTurnLeft1(robotDrive, T4_TWOS),
								new MotionMagicStraight(robotDrive,C5_TWOS),
								elevatorScaleCommand);
					}
					else{//otherwise go for opposite scale but don't drop the cube
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnRight1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS),
								new AutonTurnLeft1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS),
								elevatorScaleCommand);
					}
					}
				}
			}
			else if(m_autoSelected == Right2Cube){//if cube auton from right is selected
				if(m_prioritySelected == "Scale"){//if priority is scale
					if(gameData[1] == 'R'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_TWOS),
								new AutonTurnLeft1(robotDrive, T1_TWOS),
								new MotionMagicStraight(robotDrive, C2_TWOS),
								elevatorScaleCommand,
								new AutonDeployIntake(robotIntake),//add OpenClaw
								new AutonTurnLeft1(robotDrive,T2_TWOS),
								new MotionMagicStraight(robotDrive,C3_TWOS),
								new AutonTurnLeft1(robotDrive,T3_TWOS),
								new MotionMagicStraight(robotDrive,C4_TWOS), //closeClaw
								intakeCommand,
								new AutonStowIntake(robotIntake),
								new AutonTurnLeft1(robotDrive, T4_TWOS),
								new MotionMagicStraight(robotDrive,C5_TWOS),
								elevatorScaleCommand);
					}
					else{//otherwise go for opposite scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS),
								new AutonTurnRight1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS),
								elevatorScaleCommand);
					}

				}
				else{//if priority is switch
					if(gameData[0] == 'R'){//if switch is on right side, go for that
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_SWITCH_ONES),
								new AutonTurnLeft1(robotDrive, T1_SWITCH_ONES),
								new MotionMagicStraight(robotDrive, C2_SWITCH_ONES),
								elevatorSwitchCommand);
					}
					else{//otherwise go for a scale auton
						if(gameData[1] == 'R'){//go for same side scale
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_TWOS),
								new AutonTurnLeft1(robotDrive, T1_TWOS),
								new MotionMagicStraight(robotDrive, C2_TWOS),
								elevatorScaleCommand,
								new AutonDeployIntake(robotIntake),//add OpenClaw
								new AutonTurnLeft1(robotDrive,T2_TWOS),
								new MotionMagicStraight(robotDrive,C3_TWOS),
								new AutonTurnLeft1(robotDrive,T3_TWOS),
								new MotionMagicStraight(robotDrive,C4_TWOS), //closeClaw
								intakeCommand,
								new AutonStowIntake(robotIntake),
								new AutonTurnLeft1(robotDrive, T4_TWOS),
								new MotionMagicStraight(robotDrive,C5_TWOS),
								elevatorScaleCommand);
					}
					else{//otherwise go for opposite scale but don't drop the cube
						mainAutoCommand = AUTO_SEQUENTIAL(
								new MotionMagicStraight(robotDrive, C1_ZEROS),
								new AutonTurnLeft1(robotDrive, T1_ZEROS),
								new MotionMagicStraight(robotDrive, C2_ZEROS),
								new AutonTurnRight1(robotDrive, T2_ZEROS),
								new MotionMagicStraight(robotDrive,C3_ZEROS),
								elevatorScaleCommand);
						}
					}
				}
			}
			else{
				mainAutoCommand = AUTO_SEQUENTIAL(
						//elevatorScaleCommand
						new MotionMagicStraight(robotDrive, CL_ZEROA)
				);
			}

		}
		else {//if the game data doesn't exist for some reason default to crossing the line
			mainAutoCommand = AUTO_SEQUENTIAL(
					new MotionMagicStraight(robotDrive, CL_ZEROA));
		}

		if(mainAutoCommand) mainAutoCommand->Initialize();

	}

	void AutonomousPeriodic() {
		if(mainAutoCommand) mainAutoCommand->Run();

	}

	void TeleopInit() {
		//Initialize all the joysticks
		mainDriverStick = new Joystick(0);
		secondaryDriverStick = new Joystick(1);
		manipStick = new Joystick(2);

		//reset Drive Encoders
		robotDrive->ResetEncoders();
		robotDrive->ResetYaw();
		robotDrive->SetBrakeMode();
		robotDrive->NeutralizeDrive();

		robotDrive->FollowMode();

	}

	void TeleopPeriodic() {
		printf("Height: %f\n", robotElevator->GetElevatorPosition());


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
		robotElevator->MoveElevator(-1.0*manipStick->GetRawAxis(1));//updates elevator positions based on targets and joysticks

		if(manipStick->GetRawButton(10) || manipStick->GetRawButton(12)){
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
		/*if(manipStick->GetPOV(0) == 180){
			robotIntake->StowIntake();
		}
		else if(manipStick->GetPOV(0) == 0){
			robotIntake->DeployIntake();
		}*/
		robotIntake->BenzeneIntake(manipStick->GetPOV(0));

		//runs climber
		if(manipStick->GetRawButton(8)){
			robotClimber->SpoolClimber(true);
		}
		else{
		robotClimber->SpoolClimber(false);
		}

		if(mainDriverStick->GetRawButton(8)){
			robotDrive->TankDrive(1.0,1.0);
		}

		/*if((fabs(mainDriverStick->GetRawAxis(1))<0.01) && (fabs(mainDriverStick->GetRawAxis(1))<0.01)){
		robotDrive->TeleOpTurn(mainDriverStick->GetRawButton(12),false);
		}else{
			robotDrive->TeleOpTurn(false,true);
		}*/
		robotDrive->BenzeneDrive(throttle, -1.0 * mainDriverStick->GetRawAxis(2), mainDriverStick->GetRawButton(2));
	}
	void TestInit() {
		robotDrive->ResetEncoders();
		robotDrive->TankDrive(0,0);
	}

	// ========================================================================
	void TestPeriodic() {
		printf("Yaw %f\n", robotDrive->GetYaw());
		robotDrive->FollowMode();
		robotDrive->TankDrive(0.23,-0.23);
	}


private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();

	frc::SendableChooser<std::string> auton_chooser;
	const std::string DriveStraight= "DriveStraight";
	const std::string Center1Cube = "Center1Cube";
	const std::string Left1Cube = "Left1Cube";
	const std::string Right1Cube = "Right1Cube";
	const std::string Left2Cube = "Left2Cube";
	const std::string Right2Cube = "Right2Cube";
	const std::string Center2Cube = "Center2Cube";
	const std::string LeftSwitchOnly = "LeftSwitchOnly";
	const std::string RightSwitchOnly = "RightSwitchOnly";

	frc::SendableChooser<std::string> priority_chooser;
	const std::string Switch = "Switch";
	const std::string Scale = "Scale";

	std::string m_autoSelected;
	std::string m_prioritySelected;
};

START_ROBOT_CLASS(Robot)
