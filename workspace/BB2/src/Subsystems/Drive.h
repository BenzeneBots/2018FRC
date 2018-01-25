/*
 * Drive.h
 *
 *  Created on: Jan 23, 2018
 *      Author: sabitadhal
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_H_
#define SRC_SUBSYSTEMS_DRIVE_H_
#include <WPILib.h>


class Drive {
public:
	Drive(Victor, Victor, Victor, Victor);
	virtual ~Drive();
	void ArcadeDrive(double, double);
	void MoveToDistance(double);
	void TurnToAngle(double);
	void GetEncoderValue();
	void GetHeading();

};

#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */
