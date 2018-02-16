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

namespace Drive {

class Drive {
public:
	Drive(int,int,int,int);
	void ArcadeDrive(double, double);
	double InputScale(double, double);
	void ResetEncoders();
	double getRightEncoderValue();
	double getLeftEncoderValue();
	double getRightRate();
	double getLeftRate();
	double getLeftEncoderDistance();
	double getRightEncoderDistance();
private:
	DifferentialDrive *drivetrain;
	WPI_TalonSRX *frontLeft, *frontRight, *backLeft, *backRight;
};

} /* namespace Drive */

#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */
