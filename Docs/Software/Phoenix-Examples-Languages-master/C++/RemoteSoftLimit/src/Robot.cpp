/**
 * Example demonstrating the limit switch and remote limit switch features of CTRE Products.
 * Use a Logitech gamepad in D-Input mode.
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

	PigeonIMU & _imu = *new PigeonIMU(3);


	/* a couple latched values to detect on-press events for buttons and POV */
	bool _btns[Constants.kNumButtonsPlusOne];

	void InitRobot() {

		_motorCntrller.Set(ControlMode::PercentOutput, 0);

		/* pick directions */
		_motorCntrller.SetSensorPhase(true);
		_motorCntrller.SetInverted(false);

		/* use feedback connector but disable feature, use-webdash to reenable */
		_motorCntrller.ConfigForwardSoftLimitEnable(true, Constants.kTimeoutMs);
		_motorCntrller.ConfigReverseSoftLimitEnable(true, Constants.kTimeoutMs);

		/* speed up CANifier frames related to signals sunk by Talon/Victor */
		_canifLimits.SetStatusFramePeriod(CANifierStatusFrame::CANifierStatusFrame_Status_2_General, 10, Constants.kTimeoutMs); /* speed up quadrature pos/vel */
		_canifLimits.SetStatusFramePeriod(CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1, 10, Constants.kTimeoutMs); /* speed up PWM1 */

		/* Pick local quadrature to start with */
		SelectSoftLimitSetup(1);
	}
	/* wrapper that is close to Java */
	void Println(const char * msg) {
		printf("%s\n", msg);
	}
	/**
	 * General setup requires
	 * - configure remote filter 0 (if used)
	 * - configure remote filter 1 (if used)
	 * - select remote 0 or remote 1 sensor
	 * - pick soft limit thresholds
	 * - enable soft limit (done in InitRobot).
	 */
	void SelectSoftLimitSetup(int choice) {
		if (choice == 1) {

			/* not using remote 0 - turn it off to prevent remote LossOfSignal (LOS) fault. */
			_motorCntrller.ConfigRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource::RemoteSensorSource_Off,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* not using remote 1 - turn it off to prevent remote LossOfSignal (LOS) fault. */
			_motorCntrller.ConfigRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource::RemoteSensorSource_Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			/* select local quadrature if using Talon SRX */
			_motorCntrller.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.ConfigForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
			_motorCntrller.ConfigReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

			Println("Using local quadrature encoder.");
		} else if (choice == 2) {
			/* select a quadrature encoder connected to a remote Talon */
			_motorCntrller.ConfigRemoteFeedbackFilter(	_talonLimits.GetDeviceID(),
														RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* not using remote 1 */
			_motorCntrller.ConfigRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource::RemoteSensorSource_Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			_motorCntrller.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.ConfigForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
			_motorCntrller.ConfigReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

			Println("Using remote Talon's quadrature encoder.");
		} else if (choice == 3) {
			/* select a quadrature encoder connected to a CANifier */
			_motorCntrller.ConfigRemoteFeedbackFilter(	_canifLimits.GetDeviceNumber(),
														RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* not using remote 1 */
			_motorCntrller.ConfigRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource::RemoteSensorSource_Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			/* select remote 0 for sensor features */
			_motorCntrller.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.ConfigForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
			_motorCntrller.ConfigReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

			Println("Using remote CANifier's quadrature encoder.");
		} else if (choice == 4) {
			/* select a ribbon-cabled Pigeon that is ribbon cabled to a remote Talon. */
			_motorCntrller.ConfigRemoteFeedbackFilter(	_talonLimits.GetDeviceID(),
														RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);
			/* not using remote 1 */
			_motorCntrller.ConfigRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource::RemoteSensorSource_Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			_motorCntrller.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.ConfigForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Pigeon, Constants.kTimeoutMs);
			_motorCntrller.ConfigReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Pigeon, Constants.kTimeoutMs);

			Println("Using remote Pigeon Yaw that is plugged into a remote Talon.");
		} else if (choice == 5) {

			/* turn off remote 0 */
			_motorCntrller.ConfigRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource::RemoteSensorSource_Off,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* select a Pigeon on CAN Bus. */
			_motorCntrller.ConfigRemoteFeedbackFilter(	_imu.GetDeviceNumber(),
														RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw,
														Constants.REMOTE_1, /* use remote filter 1 this time */
														Constants.kTimeoutMs);

			_motorCntrller.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor1, /* use remote filter 1 this time */
														Constants.PID_PRIMARY, Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.ConfigForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Pigeon, Constants.kTimeoutMs);
			_motorCntrller.ConfigReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Pigeon, Constants.kTimeoutMs);

			Println("Using remote Pigeon that is CAN bus connected.");
		} else if (choice == 7) {

			/* turn off remote 0 */
			_motorCntrller.ConfigRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource::RemoteSensorSource_Off,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* select a Pigeon on CAN Bus. */
			_motorCntrller.ConfigRemoteFeedbackFilter(	_canifLimits.GetDeviceNumber(),
														RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1,
														Constants.REMOTE_1, /* use remote filter 1 this time */
														Constants.kTimeoutMs);

			_motorCntrller.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor1, /* use remote filter 1 this time */
														Constants.PID_PRIMARY, Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.ConfigForwardSoftLimitThreshold(Constants.kForwardSoftLimit_PWMInput, Constants.kTimeoutMs);
			_motorCntrller.ConfigReverseSoftLimitThreshold(Constants.kReverseSoftLimit_PWMInput, Constants.kTimeoutMs);

			Println("Using remote CANifier PWM input 1.");
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
			SelectSoftLimitSetup(1);
		}
		if (btns[2] && !_btns[2]) {
			/* if button2 is just pressed */
			SelectSoftLimitSetup(2);
		}
		if (btns[3] && !_btns[3]) {
			/* if button3 is just pressed */
			SelectSoftLimitSetup(3);
		}
		if (btns[4] && !_btns[4]) {
			/* if button4 is just pressed */
			SelectSoftLimitSetup(4);
		}
		if (btns[5] && !_btns[5]) {
			/* if button5 is just pressed */
			SelectSoftLimitSetup(5);
		}
		if (btns[7] && !_btns[7]) {
			/* if button7 is just pressed */
			SelectSoftLimitSetup(7);
		}

		if (btns[6] && !_btns[6]) {
			/* top right shoulder button - don't neutral motor if remote limit source is not available */
			int value = 1;
			_motorCntrller.ConfigSetParameter(	ParamEnum::eSoftLimitDisableNeutralOnLOS,
												value,
												0x00,
												0x00,
												Constants.kTimeoutMs);

			Println("Checking disabled for sensor presence");
		}
		if (btns[8] && !_btns[8]) {
			/* btm right shoulder button - neutral motor if remote limit source is not available */
			int value = 0;
			_motorCntrller.ConfigSetParameter(	ParamEnum::eSoftLimitDisableNeutralOnLOS,
												value,
												0x00,
												0x00,
												Constants.kTimeoutMs);

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
