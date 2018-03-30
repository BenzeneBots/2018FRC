/*
 * Intake.h
 *
 *  Created on: Jan 25, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_INTAKE_H_
#define SRC_SUBSYSTEMS_INTAKE_H_

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Talon.h>

class Intake {
public:
	Intake(int, int, int, int, int);
	void IntakeCubes();
	void OuttakeCubes();
	void StopIntake();
	void HoldIntake();
	void OpenClaw();
	void CloseClaw();
	void DeployIntake();
	void SemiDeployIntake();
	void StowIntake();
	bool IsIntakeDeployed();
	bool IsClawOpen();
	void SetIntakeStatus(bool);
	void SetClawStatus(bool);
	void BenzeneIntake(double);
private:
	Victor *intake1, *intake2;
	Solenoid *clawActuator;
	DoubleSolenoid *angleActuator;
	bool intakeDeployedStatus, clawOpenStatus;
	Timer *deployTimer;
	Timer *clawTimer;
	bool deployStatus;
	double midStatus;
};

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
