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
	void OpenClaw();
	void CloseClaw();
	void DeployIntake();
	void StowIntake();
	bool IsIntakeDeployed();
	bool IsClawOpen();
	void SetIntakeStatus(bool);
	void SetClawStatus(bool);
private:
	Victor *intake1, *intake2;
	Solenoid *clawActuator;
	DoubleSolenoid *angleActuator;
	bool intakeDeployedStatus, clawOpenStatus;
};

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
