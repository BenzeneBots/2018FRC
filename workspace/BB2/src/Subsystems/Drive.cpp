/*
 * Drive.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#include <Subsystems/Drive.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>

namespace Drive {

DifferentialDrive *drivetrain;
TalonSRX *frontLeft, *frontRight, *rearLeft, *rearRight;



Drive::Drive(int leftFrontPort,int leftBackPort, int rightFrontPort, int rightBackPort) {
	// TODO Auto-generated constructor stub

    	// front set
		ctre::phoenix::motorcontrol::can::WPI_TalonSRX frontLeft(leftFrontPort);
		ctre::phoenix::motorcontrol::can::WPI_TalonSRX rearLeft(leftBackPort);

		 // rear set
		ctre::phoenix::motorcontrol::can::WPI_TalonSRX frontRight(rightFrontPort);
		ctre::phoenix::motorcontrol::can::WPI_TalonSRX rearRight(rightBackPort);

		// speed controllers
		SpeedControllerGroup leftDrive(frontLeft, rearLeft);
		SpeedControllerGroup rightDrive{frontRight, rearRight};

		// Create drive object
		drivetrain = new DifferentialDrive (leftDrive, rightDrive);

}

} /* namespace Drive */
