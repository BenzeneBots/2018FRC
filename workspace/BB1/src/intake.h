/*
 * intake.h
 *
 *  Created on: Apr 5, 2018
 *      Author: Sabita Dhal
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <Robot.h>
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Talon.h>

void IntakeInit(DoubleSolenoid*);

void StowIntake(Solenoid*,DoubleSolenoid*);
void DeployIntake(DoubleSolenoid*);

void CloseClaw(Solenoid*);
void OpenClaw(Solenoid*);

void CheesyIntake(Victor*,double,double,bool,bool);

void BenzeneIntake(int,Solenoid*,DoubleSolenoid*,Victor*,double,double,bool,bool,bool,bool);




#endif /* SRC_INTAKE_H_ */
