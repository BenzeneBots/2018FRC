/**
 * Demo for limit switch features on Talon SRX.
 * [1] Enable robot and use gamepad Y-Axis to drive Talon.
 * [2] Press button 1 to configure limit switch to normally-open
 * [3] Press button 2 to configure limit switch to normally-closed
 * [4] Hold button3 to momentarily force disable limit switch logic in Talon.
 * [5] Self-test can be used to confirm [2], [3], and [4]
 * [6] Limit switch inputs are also polled and printed to console.
 */
#include <iostream>
#include <memory>
#include <string>
#include "ctre/Phoenix.h"
#include "WPILib.h"
#include <unistd.h>

class Robot: public frc::IterativeRobot {
public:
	TalonSRX *_srx = new TalonSRX(0);
	Joystick * _joy = new Joystick(0);
	std::stringstream _work;

	/* temps for compare */
	bool _btn1 = false;
	bool _btn2 = false;
	bool _btn3 = false;
	bool _btn4 = false;
	int _isFwdLimitSwitchClosed = 0;
	int _isRevLimitSwitchClosed = 0;

	/* nonzero to block the config until success, zero to skip checking */
	const int kTimeoutMs = 10;

	/* everytime we enter disable, reinit*/
	void DisabledInit() {
		_srx->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10); /* MagEncoder meets the requirements for Unit-Scaling */
		_srx->SetStatusFramePeriod(StatusFrame::Status_1_General_, 5, 10); /* Talon will send new frame every 5ms */
	}
	/* every loop */
	void TeleopPeriodic() {
		bool btn1 = _joy->GetRawButton(1); /* get buttons */
		bool btn2 = _joy->GetRawButton(2);
		bool btn3 = _joy->GetRawButton(3);
		bool btn4 = _joy->GetRawButton(4);
		double output = -1.0 * _joy->GetY(); /* forward is positive */
		int isFwdLimitSwitchClosed =_srx->GetSensorCollection().IsFwdLimitSwitchClosed();
		int isRevLimitSwitchClosed =_srx->GetSensorCollection().IsRevLimitSwitchClosed();

		/* on button unpress => press, change pos register */
		if (!_btn1 && btn1) {
			_srx->ConfigForwardLimitSwitchSource(
					LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
					LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
					kTimeoutMs);

			_srx->ConfigReverseLimitSwitchSource(
					LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
					LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
					kTimeoutMs);

			_work << "ConfigForwardLimitSwitchSource(FeedbackConnector, NormallyOpen)" << std::endl;
			_work << "ConfigReverseLimitSwitchSource(FeedbackConnector, NormallyOpen)" << std::endl;
		}
		/* on button unpress => press, change pos register */
		if (!_btn2 && btn2) {
			_srx->ConfigForwardLimitSwitchSource(
					LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
					LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
					kTimeoutMs);

			_srx->ConfigReverseLimitSwitchSource(
					LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
					LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
					kTimeoutMs);

			_work << "ConfigForwardLimitSwitchSource(FeedbackConnector, NormallyClosed)" << std::endl;
			_work << "ConfigReverseLimitSwitchSource(FeedbackConnector, NormallyClosed)" << std::endl;
		}

		/* button3 will override off the limit switches. */
		if (_btn3 != btn3) {
			/* only print on change */
			_work << "OverrideLimitSwitchesEnable(" << (!btn3 ? "1" : "0") << ")" << std::endl;
		}
		_srx->OverrideLimitSwitchesEnable(!btn3);/* when button is down, force off the limit switch */

		/* print any changes to limit switch input */
		if (isFwdLimitSwitchClosed != _isFwdLimitSwitchClosed) {
			_work << "isFwdLimitSwitchClosed = " << isFwdLimitSwitchClosed
					<< std::endl;
		}
		if (isRevLimitSwitchClosed != _isRevLimitSwitchClosed) {
			_work << "isRevLimitSwitchClosed = " << isRevLimitSwitchClosed
					<< std::endl;
		}

		/* direct control of motor controller */
		_srx->Set(ControlMode::PercentOutput, output);

		/* print any rendered strings, and clear work */
		printf(_work.str().c_str());
		_work.str("");

		/* cache states so we can compare next loop */
		_btn1 = btn1; /* save button states */
		_btn2 = btn2;
		_btn3 = btn3;
		_btn4 = btn4;
		_isFwdLimitSwitchClosed = isFwdLimitSwitchClosed;
		_isRevLimitSwitchClosed = isRevLimitSwitchClosed;
	}
};

START_ROBOT_CLASS(Robot)
