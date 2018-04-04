/*
 * buttons.cpp
 *
 *  Created on: Mar 26, 2018
 *      Author: James Kemp
 */

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Robot.h>
#include <buttons.h>

extern Joystick *joy;

// Read all the buttons from the joysticks.  Also, maintains a set of flags
// that go true one time on each keypress.
// ============================================================================
void ReadButtons( struct btns *b, Joystick *joy, Joystick *joy2 ) {
	static bool btnOld[ NUM_BTN ], btnOld2[ NUM_BTN ];

	// Update the button arrays with the current values from the Joysticks.
	for ( uint16_t i = 1 ; i < NUM_BTN ; ++i ) {

		// Primary Joystick
		b->btn[ i ] = joy->GetRawButton( i );
		if( b->btn[ i ] != btnOld[ i ] )
			b->btnPress[ i ] = b->btn[ i ];	// Goes true one time on press.
		else
			b->btnPress[ i ] = false;
		btnOld[ i ] = b->btn[ i ];

		// 2nd Joystick
		b->btn2[ i ] = joy2->GetRawButton( i );
		if( b->btn2[ i ] != btnOld2[ i ] )
			b->btnPress2[ i ] = b->btn2[ i ];	// Goes true one time on press.
		else
			b->btnPress2[ i ] = false;
		btnOld2[ i ] = b->btn2[ i ];
	}

	b->pov = joy->GetPOV( 0 );
	b->pov2 = joy2->GetPOV( 0 );
}

//	Based on the joystick buttons, perform actions on the claw.
// ============================================================================
void ProcessClawButtons( struct btns *b, DoubleSolenoid *cp, Solenoid *cc, Victor *it ) {

	// Claw Clamp Control: Open / Close 
	if( b->btnPress[ clawOpen ] || b->btnPress2[ clawOpen2 ] )
		cc->Set( true );
	if( b->btnPress[ clawClose ] || b->btnPress2[ clawClose2 ] )
		cc->Set( false );


#ifdef XBOX
	// Push POV up to lower claw.
	if( (b->pov == povS) || (b->pov2 == povN) )
		cp->Set( DoubleSolenoid::kForward );
	// Push POV down to raise claw.
	else if( (b->pov == povN) || (b->pov2 == povS) )
		cp->Set( DoubleSolenoid::kReverse );
	// On no POV pushed, stop the claw mid way up/down.
	else
		cp->Set( DoubleSolenoid::kOff );
#else
	// Push POV up to lower claw.
	if( (b->pov == povN) || (b->pov2 == povN) )
		cp->Set( DoubleSolenoid::kForward );
	// Push POV down to raise claw.
	else if( (b->pov == povS) || (b->pov2 == povS) )
		cp->Set( DoubleSolenoid::kReverse );
	// On no POV pushed, stop the claw mid way up/down.
	else
		cp->Set( DoubleSolenoid::kOff );
#endif

#ifdef XBOX
	double ejectSp = joy->GetRawAxis( 3 );		// Driver eject speed.
	double intakeSp = joy->GetRawAxis( 2 );		// Driver eject speed.

	// Run intake motor according to the buttons.
	if( intakeSp > 0.05 || b->btn2[ intake2 ] )
		if( intakeSp < 0.05 ) it->Set( -1.0 );
		else it->Set( intakeSp * -1.0 );
	else if ( ejectSp > 0.05 || b->btn2[ eject2 ] )
		if( ejectSp < 0.05 ) it->Set( 1.0 );
		else it->Set( ejectSp );
	else
		it->Set( 0.0 );
#else
	// Run intake motor according to the buttons.
	if( b->btn[ intake ] || b->btn2[ intake2 ] )
		//it->Set( (-1.0 * joy->GetThrottle() + 1.0) / 2.0 );
		it->Set( -1.0 );
	else if ( b->btn[ eject ] || b->btn2[ eject2 ] )
		it->Set( 1.0 );
	else
		it->Set( 0.0 );
#endif
}

