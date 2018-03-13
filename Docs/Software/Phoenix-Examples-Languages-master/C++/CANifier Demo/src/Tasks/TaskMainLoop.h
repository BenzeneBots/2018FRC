#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskMainLoop: public ctre::phoenix::tasking::ILoopable {
public:
	virtual ~TaskMainLoop();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
