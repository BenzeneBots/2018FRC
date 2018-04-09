/*
 * Init.h
 *
 *  Created on: Mar 26, 2018
 *      Author: James Kemp
 */

#ifndef SRC_INIT_H_
#define SRC_INIT_H_

#include <Robot.h>

extern gains mpGains;

void DrivetrainInit( TalonSRX *lm, TalonSRX *ls, TalonSRX *rm, TalonSRX *rs );
void AuxMotorInit( Victor *mtrClimber, Victor *mtrIntake );
void GyroInit( PigeonIMU *gyro );
void PnumaticsInit( Compressor *cp, DoubleSolenoid *clawPick, Solenoid *clawClamp );
void JoystickInit( Joystick *joy, Joystick *joy2 );
void DriverModeInit( Compressor *cp, TalonSRX *lf, TalonSRX *rt );
void setDirModeNormal( bool flgNormal, TalonSRX *lm, TalonSRX *ls, TalonSRX *rm, TalonSRX *rs  );


#endif /* SRC_INIT_H_ */
