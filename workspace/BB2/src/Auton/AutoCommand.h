/*
 * AutoCommand.h
 *
 *  Created on: Mar 7, 2018
 *      Author: Murali
 */

#ifndef SRC_AUTON_AUTOCOMMAND_H_
#define SRC_AUTON_AUTOCOMMAND_H_

#include "WPILib.h"

class AutoCommand {
public:
	AutoCommand();

	virtual void Initialize() = 0;	//called first time command runs; used to set one-time things like targets
	virtual bool Run() = 0;			//the main function of the command; returns true when its finished
	void SetTimeout(double);				//used to set a timeout for this Command
	bool IsTimeoutExpired();		//if a timeout is set, checks if it is expired yet

protected:
	double timeout;
	Timer* timer;
};

#endif /* SRC_AUTON_AUTOCOMMAND_H_ */
