/*
 * climber.cpp
 *
 */

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>
#include <climber.h>

// ============================================================================
void ProcessClimberButtons( struct btns *btns, Victor *mtrClimber ) {

	// While button pressed, driver climber up.
	if( btns->btn2[climber2] )
		mtrClimber->Set( 1.0 );
	else
		mtrClimber->Set( 0.0 );
}


