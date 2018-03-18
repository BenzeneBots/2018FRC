/*
 * Drive.h
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_H_
#define SRC_SUBSYSTEMS_DRIVE_H_

#include <WPILib.h>
#include <ctre/Phoenix.h>

class Drive {
public:
	Drive(int,int,int,int,int);
	void ArcadeDrive(double, double);
	void TankDrive(double, double);
	double InputScale(double, double);
	void ResetEncoders();
	double GetRightEncoderValue();
	double GetLeftEncoderValue();
	double GetRightVelocity();
	double GetLeftVelocity();
	double GetLeftEncoderDistance();
	double GetRightEncoderDistance();
	double GetAverageEncoderValue();
	double GetAverageVelocity();
	double GetAverageEncoderDistance();
	void ResetYaw();
	double GetYaw();
	void ResetFusedHeading();
	double GetFusedHeading();
	void SetBrakeMode();
	void SetCoastMode();
	double AutonRamping1(double,double,double,double,double);
	double AutonRamping2(double,double,double,double,double,double,double,double);
	void MotionMagicStraight(double);
	void NeutralizeDrive();
	void MyArcadeDrive(double,double,bool);
	void setDriveMtrSp(float,float);
	float fLimitVal(float,float,float);

private:
	DifferentialDrive *drivetrain;
	WPI_TalonSRX *frontLeft, *frontRight, *backLeft, *backRight;
	PigeonIMU *pidgey;
	double NormalizeAngle(double);
};

#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */
