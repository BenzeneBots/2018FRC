/*
 * AutonSCurve.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_AUTONSCURVE_H_
#define SRC_AUTON_AUTONSCURVE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonSCurve: public AutoCommand {
public:
	AutonSCurve(Drive*,int,bool);
	virtual ~AutonSCurve();

	void Initialize();
	bool Run();
private:
	Drive* drive;
	int idx;
	bool isRightSide;
	Timer *escapeTimer;
};

#endif /* SRC_AUTON_AUTONSCURVE_H_ */
