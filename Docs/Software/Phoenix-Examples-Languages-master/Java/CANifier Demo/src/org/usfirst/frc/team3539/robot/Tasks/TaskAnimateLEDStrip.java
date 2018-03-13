package org.usfirst.frc.team3539.robot.Tasks;

import com.ctre.phoenix.ILoopable;
import org.usfirst.frc.team3539.robot.Platform.Tasks;

public class TaskAnimateLEDStrip implements ILoopable {
	private float _hue;

	/* ILoopable */
	public void onStart() {

	}

	public void onStop() {

	}

	public boolean isDone() {
		return false;
	}

	public void onLoop() {
		/* Ramp through the outer rim of the HSV color wheel */
		_hue += 1;
		if (_hue >= 360) {
			_hue = 0;
		}

		/* Update LEDStrip/HSV target */
		Tasks.taskHSV_ControlLedStrip.Hue = _hue;
		Tasks.taskHSV_ControlLedStrip.Saturation = 1.0f; /*
															 * Outer rim of HSV
															 * color wheel
															 */
		Tasks.taskHSV_ControlLedStrip.Value = 0.05f; /*
														 * Hard-code the
														 * brightness
														 */
	}

	public String toString() {
		return "AnimateLEDStrip:" + _hue;
	}
}
