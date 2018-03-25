/*
 * MotionMagicTurn.h
 *
 *  Created on: Mar 17, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_MOTIONMAGICTURN_H_
#define SRC_AUTON_MOTIONMAGICTURN_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class MotionMagicTurn: public AutoCommand {
public:
	MotionMagicTurn(Drive*,double,bool);
	virtual ~MotionMagicTurn();
		void Initialize();
		bool Run();

private:
	Drive* drive;
	double ang;
	Timer* initTimer;
	double correctionAngle;
	double targetAngle;
	bool side;
};

#endif /* SRC_AUTON_MOTIONMAGICTURN_H_ */
