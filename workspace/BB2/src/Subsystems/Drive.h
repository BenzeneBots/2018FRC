/*
 * Drive.h
 *
 *  Created on: Jan 24, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_H_
#define SRC_SUBSYSTEMS_DRIVE_H_
#include <ctre/Phoenix.h>
#include <WPILib.h>
#include <Talon.h>

class Drive {
public:
	Drive(int, int, int, int);
	virtual ~Drive();
	void ArcadeDrive(double, double);

	TalonSRX *left1;
	TalonSRX *left2;
	TalonSRX *right1;
	TalonSRX *right2;

	SpeedControllerGroup *leftDrive;
	SpeedControllerGroup *rightDrive;

	DifferentialDrive *drivetrain;
};

#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */
