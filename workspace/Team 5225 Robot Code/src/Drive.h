/*
 * Drive.h
 *
 *  Created on: Jan 28, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_H_
#define SRC_SUBSYSTEMS_DRIVE_H_

#include <WPILib.h>
#include <ctre/Phoenix.h>

class Drive {
public:
	Drive(int,int,int,int);
	void ArcadeDrive(double, double);
	void TankDrive(double, double);

private:
	DifferentialDrive *drivetrain;
	WPI_TalonSRX *frontLeft, *frontRight, *backLeft, *backRight;
};

class AutonDrive : public Drive {
};


#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */


