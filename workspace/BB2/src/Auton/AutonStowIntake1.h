/*
 * AutonStowIntake1.h
 *
 *  Created on: Mar 9, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AutonStowIntake1_H_
#define SRC_AUTON_AutonStowIntake11_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Intake.h>
#include <WPILib.h>

class AutonStowIntake1: public AutoCommand {
public:
	AutonStowIntake1(Intake*,double);
	virtual ~AutonStowIntake1();

	void Initialize();
	bool Run();

private:
	Intake *intake;
	Timer* autonTimer;
	double deployPosition;
};

#endif /* SRC_AUTON_AutonStowIntake1_H_ */
