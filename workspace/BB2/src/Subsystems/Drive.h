/*
 * Drive.h
 *
 *  Created on: Jan 28, 2018
 *      Author: Murali
 */

#ifndef SRC_SUBSYSTEMS_DRIVE_H_
#define SRC_SUBSYSTEMS_DRIVE_H_
#define NUM_PATHS	6	// There are six possible paths in Auto mode.
#define MAX_WPS		10	// Max number of waypoints in a path.

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <pathfinder.h>

class Drive {
public:
	//constructor
	Drive(int,int,int,int,int);

	//drive modes
	void ArcadeDrive(double, double);
	void TankDrive(double, double);
	void BenzeneDrive(double, double, bool);

	//getters, setters, and resetters
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
	double dLimitVal(float,float,float);
	void MotionMagicTurn(double);
	void mpThread(void);
	void LoadProfile(int,bool);
	bool RunProfile(void);
	TrajectoryDuration GetTrajectoryDuration(int);
	void FindPath(int);
	void Load_Waypoints();

private:
	MotionProfileStatus mpStatus;
	DifferentialDrive *drivetrain;
	WPI_TalonSRX *frontLeft, *frontRight, *backLeft, *backRight;
	PigeonIMU *pidgey;
	bool wasStraightButtonPressed;
	double NormalizeAngle(double);

	double pos, vel,heading;	// For active traj. Pt.

	bool enXfer;
	uint32_t cntProfile;
	bool flgRunMP;

	enum paths {
		Side_Switch, Side_Scale, Side_SwitchFar,
		Side_ScaleFar, Mid_SwitchLeft, 	Mid_SwitchRight };
	typedef paths paths;

	// Struct holds a waypoint group.
	struct wp {
		Waypoint wps[ MAX_WPS ];		// Allocate Space for up to X Waypoints.
		int wpLen;						// Save how many waypoints to are use of the ten available.
		int trajLen;					// Number of points in the generated trajectory.
		double vel, accel, jerk;		// Max velocity, acceleration, and jerk for this profile.
		char sTrajLeft[32];				// Allocate space to for the Left trajectories filename.
		char sTrajRight[32];			//   "						 Right
		char sTraj_CSV[32];				// Filename for CSV data.
	} wp[NUM_PATHS];

	int trajLen;
	const char *sPath;

		// Global vars used to hold trajectory paths for left and right side.
		Segment *leftTrajectory;	// PathFinder() uses malloc to dynamically adjust.
		Segment *rightTrajectory;

		Segment leftTraj[ 2048 ];
		Segment rightTraj[ 2048 ];

};

#endif /* SRC_SUBSYSTEMS_DRIVE_H_ */
