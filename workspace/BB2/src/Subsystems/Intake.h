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

namespace Intake {

class Intake {
public:
	Intake(int, int, int);
	void IntakeCubes();
	void OuttakeCubes();
	void StopIntake();
	void DeployIntake();
	void StowIntake();

private:
	Victor *intake1, *intake2;
	TalonSRX *angleMotor;
};

} /* namespace Intake */

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
