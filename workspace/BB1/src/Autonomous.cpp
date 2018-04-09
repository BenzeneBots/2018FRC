/*
 * Autonomous.cpp
 *
 *  Created on: Apr 2, 2018
 *      Author: James Kemp
 */

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>
#include <Autonomous.h>

std::string sPos;					// Starting position on the field.
std::string sPrim, sSec, sThird;	// Primary, secondary, and third pick choices.
std::string sGame;					// This is the game data three characters.


//	Given all the options, find the best path to follow.
// ============================================================================
paths AutoFindPath() {
	if( sPos == "L" ) {
		if( sPrim == "LSW" && sGame[0] == 'L' )
			return Side_Switch;
		if( sPrim == "LSC" && sGame[1] == 'L' )
			return Side_Scale;
	}

	return NA;
}

//	Return the starting position as a enum.
// ============================================================================
startPos AutoGetStartPos() {
	if( sPos == "L" ) return left;
	if( sPos == "M" ) return mid;
	if( sPos == "R" ) return right;
	return right;	// Default
}

//	Init all the stuff needed for autonomous mode.
// ============================================================================
void AutoInit( TalonSRX *lf, TalonSRX *rt, PigeonIMU *gyro ) {

	printf( "AutoInit...\n" );

	/*
	sPos = SmartDashboard::GetString( "DB/String 0", "na" );	// Starting Location
	sPrim = SmartDashboard::GetString( "DB/String 1", "na" );	// Primary Move
	sSec = SmartDashboard::GetString( "DB/String 2", "na" );	// Secondary Move
	sThird = SmartDashboard::GetString( "DB/String 3", "na" );	// Third Pick Move
	sGame = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	printf( "Position: %s\n", sPos.c_str() );
	printf( "Primary Auto: %s\n", sPrim.c_str() );
	printf( "Second Auto: %s\n", sSec.c_str() );
	printf( "Third Pick Auto: %s\n", sThird.c_str() );
	printf( "Auto Game Data: %c%c%c\n", sGame[0], sGame[1], sGame[2] );
	*/

	lf->NeutralOutput();
	rt->NeutralOutput();

	lf->SetSelectedSensorPosition( 0, 0, 0 );
	rt->SetSelectedSensorPosition( 0, 0, 0 );
	gyro->SetFusedHeading( 0.0, 0 );

	lf->SetIntegralAccumulator( 0.0, 0, 0 );
	rt->SetIntegralAccumulator( 0.0, 0, 0 );

	lf->ClearMotionProfileTrajectories();
	rt->ClearMotionProfileTrajectories();
}
