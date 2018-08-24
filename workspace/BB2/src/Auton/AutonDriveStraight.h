/*
 * AutonDriveStraight.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONDRIVESTRAIGHT_H_
#define SRC_AUTON_AUTONDRIVESTRAIGHT_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonDriveStraight: public AutoCommand {
public:
	AutonDriveStraight(Drive*, double);
	virtual ~AutonDriveStraight();

	void Initialize();
	bool Run();

private:
	Drive *drive;
	double distance, startYaw;
	Timer *frictionTimer;
};

#endif /* SRC_AUTON_AUTONDRIVESTRAIGHT_H_ */
