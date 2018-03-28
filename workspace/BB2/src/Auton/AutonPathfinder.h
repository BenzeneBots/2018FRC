/*
 * AutonPathfinder.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTONPATHFINDER_H_
#define SRC_AUTON_AUTONPATHFINDER_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonPathfinder: public AutoCommand {
public:
	AutonPathfinder(Drive*, int, bool);
	virtual ~AutonPathfinder();

	void Initialize();
	bool Run();

private:
	Drive *drive;
	int profileId;
	bool isReversed;
};

#endif /* SRC_AUTON_AUTONPATHFINDER_H_ */
