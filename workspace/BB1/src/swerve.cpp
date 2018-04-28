/*
 * swerve.cpp
 *
 *  Created on: Apr 28, 2018
 *      Author: Rahul Naik
 */

#include "math.h"

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>

#define wheelbase 30 //length of drivebase
#define trackwidth 24 //width of drivebase

#define PI 3.14159

class SwerveRobot : public TimedRobot {
private:
public:

  TalonSRX *FLdrive, *FLsteer, *RLdrive, *RLsteer, *FRdrive, *FRsteer, *RRdrive, *RRsteer;
  Joystick *driveJoy;
  PigeonIMU *gyro;

  void RobotInit() {
      FLdrive = new TalonSRX(1);
      FLsteer = new TalonSRX(2);
      RLdrive = new TalonSRX(3);
      RLsteer = new TalonSRX(4);
      FRdrive = new TalonSRX(5);
      FRsteer = new TalonSRX(6);
      RRdrive = new TalonSRX(7);
      RRsteer = new TalonSRX(8);
      driveJoy = new Joystick(1);
      gyro = new PigeonIMU(0);
  }

  void operatorControl() {
    	double fwd;
    	double str;
    	double rcw;
    	double temp;
    	double fwd2;
    	double str2;
    	double r;
    	double a;
    	double b;
    	double c;
    	double d;
    	double frs, fls, rls, rrs; //Front Right, Front Left, Rear Left, Rear Right Wheel Speeds
    	double fra, fla, rla, rra; //Wheel Angles
    	double max;
    	double compass;

    	fwd = driveJoy->GetRawAxis( 1 );
    	str = driveJoy->GetRawAxis( 2 );
    	rcw = driveJoy->GetRawAxis( 3 );

    	compass = gyro->GetCompassHeading();
    	temp = fwd*cos(compass) + str*sin(compass);
    	str2 = -fwd*sin(compass) + str*cos(compass);
    	fwd2 = temp;

    	r = sqrt((wheelbase*wheelbase) + (trackwidth*trackwidth));

    	a = str2 - rcw * (wheelbase/r);
    	b = str2 + rcw * (wheelbase/r);
    	c = fwd2 - rcw * (trackwidth/r);
    	d = fwd2 + rcw * (trackwidth/r);

    	frs = sqrt(b*b + c*c);
    	fls = sqrt(b*b + d*d);
    	rls = sqrt(a*a + d*d);
    	rrs = sqrt(a*a + c*c);

    	fra = atan2(b,c) * 180/PI;
    	fla = atan2(b,d) * 180/PI;
    	rra = atan2(a,d) * 180/PI;
    	rla = atan2(a,c) * 180/PI;

    	//Because the Talon's position control uses values from 0 - 1023
    	//for potentiometer ranges, we must modify the wheel angles to
    	//compensate for this. To do this add 180 to the wheel angle to
    	//get a 0-360 range then multiply by 1023/360 which equals 2.8444...

    	FRsteer->Set(ControlMode::Position, (fra+180) * (1023/360));
    	FLsteer->Set(ControlMode::Position, (fla+180) * (1023/360));
    	RLsteer->Set(ControlMode::Position, (rla+180) * (1023/360));
    	RRsteer->Set(ControlMode::Position, (rra+180) * (1023/360));

    	//Normalize wheel speeds
    	max = frs;
    	if(fls>max){
    		max = fls;
    	}
    	if(rls>max){
    		max = rls;
    	}
    	if(rrs>max){
    		max = rrs;
    	}
    	if(max>1){
    		frs/=max;
    		fls/=max;
    		rrs/=max;
    		rls/=max;
    	}
    	//Wheel speeds are now 0-1. Not -1 to +1.
    	FRdrive->Set(ControlMode::PercentOutput, frs);
    	FLdrive->Set(ControlMode::PercentOutput, fls);
    	RRdrive->Set(ControlMode::PercentOutput, rrs);
    	RLdrive->Set(ControlMode::PercentOutput, rls);
    }
};

