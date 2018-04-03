/*
 * utils.h
 *
 *  Created on: Mar 26, 2018
 *      Author: James Kemp
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>


void UpdateSmartDash( PigeonIMU *gyro, TalonSRX *lf, TalonSRX *rt );
void RotateBase( TalonSRX *lf, TalonSRX *rt, PigeonIMU *gyro );
void RebuildMotionProfiles( void );


#endif /* SRC_UTILS_H_ */
