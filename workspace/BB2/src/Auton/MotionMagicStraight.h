/*
 * MotionMagicStraight.h
 *
 *  Created on: Mar 17, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_MOTIONMAGICSTRAIGHT_H_
#define SRC_AUTON_MOTIONMAGICSTRAIGHT_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>
#include <WPILib.h>

class MotionMagicStraight: public AutoCommand {
public:
	MotionMagicStraight(Drive*,double);
	virtual ~MotionMagicStraight();
		void Initialize();
		bool Run();

private:
	Drive* drive;
	double dist;
	Timer* initTimer;
};

#endif /* SRC_AUTON_MOTIONMAGICSTRAIGHT_H_ */
