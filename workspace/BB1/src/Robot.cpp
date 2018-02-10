

	#include <WPILib.h>
	#include <ctre/Phoenix.h>
	class Robot : public TimedRobot {
			/* robot peripherals */
		PigeonIMU * _pidgey;
	public:
		Robot() {
		_pidgey = new PigeonIMU(0);
		}
	};
