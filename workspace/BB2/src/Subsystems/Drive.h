/*
 * Drive.h
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_H_
#define SRC_SUBSYSTEMS_DRIVE_H_

namespace Drive {

class Drive {
public:
	Drive(int,int,int,int);
	void ArcadeDrive(double, double);
	void ResetEncoders();
	double getRightEncoderValue();
	double getLeftEncoderValue();
	double getRightRate();
	double getLeftRate();
};

} /* namespace Drive */

#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */
