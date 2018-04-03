/*
 * buttons.h
 *
 *  Created on: Mar 26, 2018
 *      Author: James Kemp
 */

#ifndef SRC_BUTTONS_H_
#define SRC_BUTTONS_H_

#include <Robot.h>

void ProcessClawButtons( struct btns *b, DoubleSolenoid *cPick, Solenoid *cClamp, Victor *mtrIntake );
void ReadButtons( struct btns *b, Joystick *joy, Joystick *joy2 );




#endif /* SRC_BUTTONS_H_ */
