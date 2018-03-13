/*
 * SequentialCommand.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Murali
 */

#include "Auton/SequentialCommand.h"

#include <stdarg.h>

SequentialCommand::SequentialCommand(int dummy, ...) {
  va_list vl;
  va_start(vl, dummy);

  AutoCommand* command = NULL;
  while (command = va_arg(vl, AutoCommand*)) {//iterates over every element in the args until it hits a null
    commandsList.push_back(command);
  }
  va_end(vl);			//stops looking at the args list


  commandIndex = 0;	//since we want to start with the first command
}

SequentialCommand::~SequentialCommand() {
  for (std::vector<AutoCommand*>::const_iterator it = commandsList.begin(); it < commandsList.end(); ++it) {
    delete *it;
  }
}

void SequentialCommand::Initialize() {
  // Only initialize the first command
  if (commandsList.size()>0) {
    commandsList[0]->Initialize();
  }
}

bool SequentialCommand::Run() {
  if (commandIndex == (int)commandsList.size()) {// If all the commands are done, the SequentialCommand is done
    return true;
  } else if (commandsList[commandIndex]->Run()) {//If the current command has finished, move on to the next one and adjust the index
    commandIndex++;
    if (commandIndex == (int)commandsList.size()) {// if there are no more commands, the SequentialCommand is done
      return true;
    } else {//otherwise initializes the next command
      commandsList[commandIndex]->Initialize();
    }
  }

  // If nothing above returned, it means that the current command isn't finished yet
  return false;
}

void SequentialCommand::AddCommand(AutoCommand* command) {
  commandsList.push_back(command);
}
