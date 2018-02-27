/*
 * Drive.h
 *
 *  Created on: Feb 26, 2018
 *      Author: Sanket Nayak
 */
#include <ctre/Phoenix.h>
#include <WPILib.h>

#ifndef SRC_DRIVE_H_
#define SRC_DRIVE_H_

class Drive {
public:
	Drive(int,int,int,int);
	void TankDrive(double, double);
private:
	DifferentialDrive *drivetrain;
	WPI_TalonSRX *frontLeft, *frontRight, *backLeft, *backRight;
};

#endif /* SRC_DRIVE_H_ */
