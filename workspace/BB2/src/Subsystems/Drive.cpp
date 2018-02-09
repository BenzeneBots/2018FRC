/*
 * Drive.cpp
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#include <Subsystems/Drive.h>
#include <ctre/Phoenix.h>
#include <WPILib.h>



//Defined drive constants
#define INCHES_PER_TICK 0.01410; //TODO measure this guy

//Defined functions
#define max( A, B )				A > B ? A : B
#define min( A, B )				A < B ? A : B
#define LimitVal( L, T, H)		max( min( T, H ), L)


namespace Drive {

DifferentialDrive *drivetrain;
Victor *frontLeft, *frontRight, *backLeft, *backRight;



Drive::Drive(int frontLeftPort,int backLeftPort, int frontRightPort, int backRightPort) {
	// TODO Auto-generated constructor stub

    // Create all drive motors
	frontLeft = new Victor(frontLeftPort);
	backLeft = new Victor(backLeftPort);
	frontRight = new Victor(frontRightPort);
	backRight = new Victor(backRightPort);
}

void Drive::ArcadeDrive(double speed, double turn){//Drives the drivetrain based on
	double leftSpeed = LimitVal( -127, (turn- speed)/2, 127 );
	double rightSpeed = LimitVal( -127, (turn + speed)/2, 127 );

	frontLeft->Set(leftSpeed);
	backLeft->Set(leftSpeed);

	frontRight->Set(rightSpeed);
	backRight->Set(rightSpeed);

	}
}
	//frontLeft->Set(ControlMode::PercentOutput, leftSpeed);
	//backLeft->Set(ControlMode::Follower, frontLeft->GetDeviceID());

	//frontRight->Set(ControlMode::PercentOutput, rightSpeed);
	//backLeft->Set(ControlMode::Follower, frontRight->GetDeviceID());

/*
void Drive::ResetEncoders(){//Resets Encoders to 0
	frontRight->GetSensorCollection().SetQuadraturePosition(0, 0); //should the timeoutMS be 0?
	frontLeft->GetSensorCollection().SetQuadraturePosition(0, 0);
}

double getRightEncoderValue(){//returns inches travelled by right side of drivetrain
	double rawEncVal = frontRight->GetSensorCollection().GetQuadraturePosition();
	double rightDistanceVal = rawEncVal * INCHES_PER_TICK;
	return rightDistanceVal;
}

double getLeftEncoderValue(){//returns inches travelled by left side of drivetrain
	double rawEncVal = frontLeft->GetSensorCollection().GetQuadraturePosition();
	double leftDistanceVal = rawEncVal * INCHES_PER_TICK;
	return leftDistanceVal;
}

double getRightRate(){//gets inches per second travelled by left side of drivetrain
	double rate = frontRight->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

double getLeftRate(){//gets inches per second travelled by right side of drivetrain
	double rate = frontLeft->GetSensorCollection().GetQuadratureVelocity();
	double VelocityVal = rate * INCHES_PER_TICK;
	return VelocityVal;
}

}  namespace Drive */

//*/
