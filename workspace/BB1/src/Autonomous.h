/*
 * Autonomous.h
 *
 *  Created on: Apr 2, 2018
 *      Author: James Kemp
 */

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

paths AutoFindPath();
void AutoInit( TalonSRX *lf, TalonSRX *rt, PigeonIMU *gyro );
startPos AutoGetStartPos();




#endif /* SRC_AUTONOMOUS_H_ */
