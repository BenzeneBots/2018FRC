/*
 * elevator.h
 *
 *  Created on: Mar 26, 2018
 *      Author: James Kemp
 */

#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_

void DriveElevator( double sp, TalonSRX *mtr, DigitalInput *sw, struct btns *btns);
void InitElevator( TalonSRX *mtr );
bool resetElevatorPos( TalonSRX *mtr, DigitalInput *sw );
void setElevatorPos( int pos );



#endif /* SRC_ELEVATOR_H_ */
