#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include "ctre/Phoenix.h"
#include "WPILib.h"
#include "Constants.h"

class Robot: public frc::IterativeRobot {
private:
	TalonSRX * _talon = new TalonSRX(0);
	Joystick * _joy = new Joystick(0);
	std::string _sb;
	int _loops = 0;

	void RobotInit() {
		/* set the peak and nominal outputs, 12V means full */
		_talon->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon->ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		_talon->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon->Config_kP(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = _joy->GetY();
		double motorOutput = _talon->GetMotorOutputPercent();
		bool button1 = _joy->GetRawButton(1);

		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tcur:");
		_sb.append(std::to_string(_talon->GetOutputCurrent()));
		/* on button1 press enter closed-loop mode on target position */
		if (button1) {
			/* Position mode - button just pressed */
			_talon->Set(ControlMode::Current, leftYstick * 40); /* 40 Amps in either direction */
		} else {
			_talon->Set(ControlMode::PercentOutput, leftYstick);
		}
		/* if Talon is in position closed-loop, print some more info */
		if (_talon->GetControlMode() == ControlMode::Current) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(_talon->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(leftYstick * 40));
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n", _sb.c_str());
		}
		_sb.clear();
	}
};

START_ROBOT_CLASS(Robot)
