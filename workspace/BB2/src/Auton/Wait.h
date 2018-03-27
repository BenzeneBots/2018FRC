/*
 * Wait.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Sabita Dhal
 */

#ifndef SRC_AUTON_WAIT_H_
#define SRC_AUTON_WAIT_H_

#include <Auton/AutoCommand.h>
#include <WPILib.h>

class Wait: public AutoCommand {
public:
	Wait(double);
	virtual ~Wait();
	void Initialize();
	bool Run();
private:
	Timer* waitTimer;
	double timeOut;
};

#endif /* SRC_AUTON_WAIT_H_ */
