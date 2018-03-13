/*
 * PathFinderDrive.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: Sanket Nayak
 */

#include <Auton/PathFinderDrive.h>
#include <pathfinder.h>

PathFinderDrive::PathFinderDrive(Drive* robotDrive) {
	int POINT_LENGTH = 3;

	Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

	Waypoint p1 = { -4, -1, d2r(45) };      // Waypoint @ x=-4, y=-1, exit angle=45 degrees
	Waypoint p2 = { -1, 2, 0 };             // Waypoint @ x=-1, y= 2, exit angle= 0 radians
	Waypoint p3 = {  2, 4, 0 };             // Waypoint @ x= 2, y= 4, exit angle= 0 radians
	points[0] = p1;
	points[1] = p2;
	points[2] = p3;

	TrajectoryCandidate candidate;
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, 0.001, 15.0, 10.0, 60.0, &candidate);

	int length = candidate.length;
	Segment *trajectory = malloc(length * sizeof(Segment));

	pathfinder_generate(&candidate, trajectory);

	Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

	double wheelbase_width = 0.6;


	EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
	EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());

	left.configureEncoder(robotDrive->GetLeftEncoderDistance(), 4096, 6);
	right.configureEncoder(robotDrive->GetRightEncoderDistance(), 4096, 6);

	left.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);
	right.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);

	double output = left.calculate(encoder_position);

	double l = left.calculate(encoder_position_left);
	double r = right.calculate(encoder_position_right);

	double gyro_heading = robotDrive->GetYaw();  // Assuming the gyro is giving a value in degrees
	double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

	double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
	double turn = 0.8 * (-1.0/80.0) * angleDifference;

	robotDrive->TankDrive((l + turn), (r - turn));

	free(trajectory);
}

