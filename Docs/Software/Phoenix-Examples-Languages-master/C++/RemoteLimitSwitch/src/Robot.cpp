/**
 * Example demonstrating the limit switch and remote limit switch features of CTRE Products.
 * This example shows
 * - using the local limit switch features of the Talon
 * - using another Talon's limit switch inputs to limit switch the motor output.
 * - using another CANifier limit switch inputs to limit switch the motor output
 */
#include <iostream>
#include <string>
#include <unistd.h>
#include <IterativeRobot.h>
#include "ctre/Phoenix.h"
#include "Joystick.h"
#include "Constants.h"

class Robot: public frc::IterativeRobot {
public:

	/* hardware objects - use references instead of pointers to match Java examples. */
	TalonSRX & _motorCntrller = *new TalonSRX(2); // could also be Victor SPX if using remote sensor features.

	CANifier & _canifLimits = *new CANifier(2); /* use this CANifier for limit switches */
	TalonSRX & _talonLimits = *new TalonSRX(5); /* use this Talon for limit switches */

	Joystick &_joy = *new Joystick(0);

	/* a couple latched values to detect on-press events for buttons and POV */
	bool _btns[Constants.kNumButtonsPlusOne];

	void InitRobot() {

		_motorCntrller.Set(ControlMode::PercentOutput, 0);

		/* pick directions */
		_motorCntrller.SetSensorPhase(false);
		_motorCntrller.SetInverted(false);

	}
	/* wrapper that is close to Java */
	void Println(const char * msg)
	{
		printf("%s\n",msg);
	}
	void SelectLimitSwitch(int choice) {
		if (choice == 0) {
			/* use feedback connector but disable feature, use-webdash to reenable */
			_motorCntrller.ConfigForwardLimitSwitchSource(	LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
													LimitSwitchNormal::LimitSwitchNormal_Disabled,
													Constants.kTimeoutMs);

			_motorCntrller.ConfigReverseLimitSwitchSource(	LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
													LimitSwitchNormal::LimitSwitchNormal_Disabled,
													Constants.kTimeoutMs);

			Println("Limit Switches disabled.");
		} else if (choice == 1) {
			/* use feedback connector - use three functions */
			_motorCntrller.ConfigForwardLimitSwitchSource(	LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
													LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
													Constants.kTimeoutMs);

			_motorCntrller.ConfigReverseLimitSwitchSource(	LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
													LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
													Constants.kTimeoutMs);

			Println("Limit Switches locally enabled.");
		} else if (choice == 2) {
			/* use remote CANifier - use four param functions */
			_motorCntrller.ConfigForwardLimitSwitchSource(	RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteCANifier,
													LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
													_canifLimits.GetDeviceNumber(),
													Constants.kTimeoutMs);

			_motorCntrller.ConfigReverseLimitSwitchSource(	RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteCANifier,
													LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
													_canifLimits.GetDeviceNumber(),
													Constants.kTimeoutMs);

			Println("Remote Limit Switches enabled using CANifier.");
		}  else if (choice == 3) {
			/* use remote Talon - use four param functions */
			_motorCntrller.ConfigForwardLimitSwitchSource(	RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
													LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
													_talonLimits.GetDeviceID(),
													Constants.kTimeoutMs);

			_motorCntrller.ConfigReverseLimitSwitchSource(	RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
													LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
													_talonLimits.GetDeviceID(),
													Constants.kTimeoutMs);

			Println("Remote Limit Switches enabled using another Talon SRX.");
		}
	}

	void CommonLoop() {

		/* grab the joystick inputs */
		bool btns[Constants.kNumButtonsPlusOne];
		GetButtons(btns);

		double joyForward = -1 * _joy.GetY(); /* positive stick => forward */

		/* deadband the sticks */
		joyForward = Deadband(joyForward);

		/* button 1*/
		if (btns[1] && !_btns[1]) {
			/* if button1 is just pressed */
			SelectLimitSwitch(1);
		}
		if (btns[2] && !_btns[2]) {
			/* if button2 is just pressed */
			SelectLimitSwitch(2);
		}
		if (btns[3] && !_btns[3]) {
			/* if button3 is just pressed */
			SelectLimitSwitch(3);
		}
		if (btns[4] && !_btns[4]) {
			/* if button4 is just pressed */
			SelectLimitSwitch(4);
		}
		if (btns[6] && !_btns[6]) {
			/* top right shoulder button - don't neutral motor if remote limit source is not available */
			int value = 1;
			_motorCntrller.ConfigSetParameter(ParamEnum::eLimitSwitchDisableNeutralOnLOS, value, 0x00, 0x00, Constants.kTimeoutMs);

			Println("Checking disabled for sensor presence");
		}
		if (btns[8] && !_btns[8]) {
			/* btm right shoulder button - neutral motor if remote limit source is not available */
			int value = 0;
			_motorCntrller.ConfigSetParameter(ParamEnum::eLimitSwitchDisableNeutralOnLOS, value, 0x00, 0x00, Constants.kTimeoutMs);

			Println("Checking enabled for sensor presence");
		}
		CopyButtons(_btns, btns);

		/* drive talon with gamepad */
		_motorCntrller.Set(ControlMode::PercentOutput, joyForward);
	}

	//------------------------- Loops -------------------------------//
	void DisabledInit() {
		/* initialize hardware so that sensor phases on set before Teleop.
		 * This makes self-test more useful. */
		InitRobot();
	}
	void DisabledPeriodic() {
		CommonLoop();
	}
	void TeleopInit() {
		/* initialize hardware at start of teleop, just in case Talon was replaced / field-upgraded during disable.
		 * All params are persistent except for status frame periods. */
		InitRobot();
	}
	void TeleopPeriodic() {
		CommonLoop();
	}

	//-------------- Some helpful routines ---------------//
	void GetButtons(bool * btns) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			btns[i] = _joy.GetRawButton(i);
		}
	}
	void CopyButtons(bool * destination, const bool * source) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			destination[i] = source[i];
		}
	}
	double Deadband(double value) {
		if (value >= +0.05) {
			return value;
		}
		if (value <= -0.05) {
			return value;
		}
		return 0;
	}
};

START_ROBOT_CLASS(Robot)
