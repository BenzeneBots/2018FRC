/*
 * AutonSCurve.h
 *
 *  Created on: Mar 17, 2018
 *      Author: Sanket Nayak
 */

#ifndef SRC_AUTON_AUTONSCURVE_H_
#define SRC_AUTON_AUTONSCURVE_H_

#include <Auton/AutoCommand.h>
#include <Subsystems/Drive.h>

class AutonSCurve: public AutoCommand {
public:
	AutonSCurve(Drive*,double,double,double,double,double,double);
	virtual ~AutonSCurve();


	void Initialize();
	bool Run();

	private:
		Drive *drive;
		double X1Dist, Y1Dist, Ang1, X2Dist, Y2Dist, Ang2;

};

#endif /* SRC_AUTON_AUTONSCURVE_H_ */
