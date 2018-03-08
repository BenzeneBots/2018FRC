/*
 * AutoCommand.cpp
 *
 *  Created on: Mar 7, 2018
 *      Author: Murali
 */

#include <Auton/AutoCommand.h>

AutoCommand::AutoCommand() {
	timer = new Timer();
	timeout = -1;
}

void AutoCommand::Initialize(){
	timer->Start();
}

void AutoCommand::SetTimeout(double newTimeout){
	timeout = newTimeout;
}

bool AutoCommand::IsTimeoutExpired() {
  if (timeout != -1) {
    return timer->Get() > timeout;
  }
  return false;
}
