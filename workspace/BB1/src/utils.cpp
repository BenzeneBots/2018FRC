/*
 * utils.cpp
 *
 *  Created on: Mar 18, 2018
 *      Author: James Kemp
 */
#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>

extern void Load_Waypoints( void );
extern void PathFinder( int idx );


//	This will step through all the motion profiles and rebuild waypoints
// into trajectory points.  The new trajectory points are stored on the 
// RoboRIO file-system.
// ============================================================================
void RebuildMotionProfiles( void ) {

	printf( "Recalculating Motion Trajectories...\n" );

	Load_Waypoints();	// Load all the data into the waypoint structures.

	// Step thru and calculate each path.  The result is stored as a 
	// binary/CSV file on the RoboRIO file-system.
	for ( int idx=0 ; idx < NUM_PATHS ; idx++ ) {
		printf( "Calculating PathFinder Path: %d\n", idx );
		PathFinder( idx );
	}
	printf( "Info: PathFinder Done\n" );
}



// ============================================================================
void RotateBase( TalonSRX *lf, TalonSRX *rt, PigeonIMU *gyro ) {

	/*
	if( btn[ twelve ] ) {
		// On button first pressed...
		if( btn12_Old == false ) {
			btn12_Old = true;
			tarHeading = heading + 90.0;
			// Default turning speed of 35%.
			lf=-0.35; rt=0.35;
			flgHold = false;	// Reset holding flag at end of turn.
			printf( "Heading: %0.2f\n", heading );
			printf( "Target Heading: %0.2f\n", tarHeading );
		}

		// Wait for turn to get with 17deg of target.  Or, we're trying to
		// hold the final position.
		if( heading >= (tarHeading - 35.0) || flgHold ) {
			// On first time trying to hold...
			if( flgHold == false ) {
				flgHold = true;
				// Set default motor outputs depending on which side of the
				// target we're on.
				if( heading > tarHeading ) {
					lf = 0.1;	// Overshot, so reverse.
					rt = -0.1;
				}
				else {
					lf = -0.1;	// Undershot, so forward.
					rt = 0.1;
				}
			}

			// Add some integral over time to the output depending on
			// which side we're on.
			if( heading > tarHeading ) {
				lf += 0.005;
				rt -= 0.005;
			}
			else {
				lf -= 0.005;
				rt += 0.005;
			}

			// Finally, if we're within 3deg, stop.
			if( fabs(fabs( heading ) - fabs( tarHeading )) < 3.0 ) {
				lf = 0.0;
				rt = 0.0;
				flgHold = false;
			}

		}

		// Set the motor outputs.
		mtrLMaster->Set( ControlMode::PercentOutput, lf );
		mtrRMaster->Set( ControlMode::PercentOutput, rt );
	}
	else {
		btn12_Old = false;
		flgHold = false;
		ArcadeDrive();
	}
	*/
}


// ============================================================================
void UpdateSmartDash( PigeonIMU *gyro, TalonSRX *lf, TalonSRX *rt ) {

	SmartDashboard::PutNumber( "chartOne", gyro->GetFusedHeading() * 100 );
	SmartDashboard::PutNumber( "chartTwo", lf->GetSelectedSensorVelocity(0) );
	SmartDashboard::PutNumber( "chartThree", rt->GetSelectedSensorVelocity( 0 ) );

	SmartDashboard::PutNumber( "imuHeading", gyro->GetFusedHeading() );
}


