/**
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */

#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include "ctre/Phoenix.h"
#include "Drive/DifferentialDrive.h"
#include "Joystick.h"

class Robot: public frc::IterativeRobot {
public:
	/* ------ [1] Update CAN Device IDs and switch to WPI_VictorSPX where necessary ------*/
	WPI_TalonSRX * _rghtFront = new WPI_TalonSRX(3);
	WPI_TalonSRX * _rghtFollower = new WPI_TalonSRX(2);
	WPI_TalonSRX * _leftFront = new WPI_TalonSRX(6);
	WPI_TalonSRX * _leftFollower = new WPI_TalonSRX(4);

	DifferentialDrive * _diffDrive = new DifferentialDrive(*_leftFront,
			*_rghtFront);

	Joystick * _joystick = new Joystick(0);

	Faults _faults_L;
	Faults _faults_R;

	void TeleopPeriodic() {

		std::stringstream work;

		/* get gamepad stick values */
		double forw = -1 * _joystick->GetRawAxis(1); /* positive is forward */
		double turn = +1 * _joystick->GetRawAxis(4); /* positive is right */

		/* deadband gamepad 10%*/
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		/* drive robot */
		_diffDrive->ArcadeDrive(forw, turn, false);

		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		work << " GF:" << forw << " GT:" << turn;

		/* get sensor values */
		//double leftPos = _leftFront->GetSelectedSensorPosition(0);
		//double rghtPos = _rghtFront->GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = _leftFront->GetSelectedSensorVelocity(0);
		double rghtVelUnitsPer100ms = _rghtFront->GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

		/* drive motor at least 25%, Talons will auto-detect if sensor is out of phase */
		_leftFront->GetFaults(_faults_L);
		_rghtFront->GetFaults(_faults_R);

		if (_faults_L.SensorOutOfPhase) {
			work << " L sensor is out of phase";
		}
		if (_faults_R.SensorOutOfPhase) {
			work << " R sensor is out of phase";
		}

		/* print to console */
		std::cout << work.str() << std::endl;
	}

	void RobotInit() {

		_rghtFollower->Follow(*_rghtFront);
		_leftFollower->Follow(*_leftFront);

		/* [3] Adjust inverts so all motor drive in the correction direction */
		_rghtFront->SetInverted(false);
		_rghtFollower->SetInverted(false);
		_leftFront->SetInverted(false);
		_leftFollower->SetInverted(false);

		/* [4] adjust sensor phase so sensor moves
		 * positive when Talon LEDs are green */
		_rghtFront->SetSensorPhase(true);
		_leftFront->SetSensorPhase(true);
	}

private:
};

START_ROBOT_CLASS(Robot)
