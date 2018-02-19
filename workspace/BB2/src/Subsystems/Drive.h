/*
 * Drive.h
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
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
	double InputScale(double, double);
	void ResetEncoders();
	double GetRightEncoderValue();
	double GetLeftEncoderValue();
	double GetRightRate();
	double GetLeftRate();
	double GetLeftEncoderDistance();
	double GetRightEncoderDistance();
private:
	DifferentialDrive *drivetrain;
	WPI_TalonSRX *frontLeft, *frontRight, *backLeft, *backRight;
};

class AutonDrive : public Drive {
};


#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */
