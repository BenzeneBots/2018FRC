/*
 * ElevateDrive.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Sabita Dhal
 */

#ifndef SRC_AUTON_ELEVATEDRIVE_H_
#define SRC_AUTON_ELEVATEDRIVE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>
#include <Subsystems/Elevator.h>
#include <WPILib.h>

class ElevateDrive: public AutoCommand {
public:
	ElevateDrive(Drive*,double,Elevator*, double);
	virtual ~ElevateDrive();
	void Initialize();
	bool Run();

private:
	Drive* drive;
	double dist;
	Timer* initTimer;
	Elevator *elevator;
	double targetHeight;
};

#endif /* SRC_AUTON_ELEVATEDRIVE_H_ */
