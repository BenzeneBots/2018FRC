/*
 * SequentialCommand.h
 *
 *  Created on: Mar 8, 2018
 *      Author: Murali
 */

#ifndef AUTO_SEQUENTIAL_COMMAND_H_
#define AUTO_SEQUENTIAL_COMMAND_H_

#include <vector>
#include <utility>

#include "Auton/AutoCommand.h"

// Macro to wrap the SequentialCommand constructor for convenience.
#define AUTO_SEQUENTIAL(...) new SequentialCommand(0, __VA_ARGS__, NULL)


class SequentialCommand : public AutoCommand {
 public:

  SequentialCommand(int dummy, ...);		//constructor - usually called through above macro
  virtual ~SequentialCommand();				//destructor

  virtual void Initialize();				//Initializes JUST the first command
  virtual bool Run();						//Runs the next command in queue
  void AddCommand(AutoCommand* command);	//Adds command to list


 private:
  std::vector<AutoCommand*> commandsList;
  int commandIndex; // the index of the current command being executed
};

#endif  // AUTO_SEQUENTIAL_COMMAND_H_
