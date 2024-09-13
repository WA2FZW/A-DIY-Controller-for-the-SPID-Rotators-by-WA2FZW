/*
 * 	SPID.h - Version 1.2
 *
 *	This header file is used by the SPID rotator controller Version 1.2 software.
 *
 *	The variables defined in here might need to be tweaked based on the characteristics
 *	of any specific rotator and/or hardware used to build the smart controller.
 *	Those variables and explanations of why and how to change them are included here.
 *
 *	Definitions in the actual program file should NOT be changed.
 */

#ifndef SPID_H						// Prevent double #include
#define SPID_H


/*
 *	Libraries needed and where to get the non-standard ones:
 */

#include <LiquidCrystal_I2C.h>		// From https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include <SoftReset.h>				// From https://github.com/WickedDevice/SoftReset
#include <EEPROM.h>					// Built-in
#include <Wire.h> 					// Built-in
#include <String.h>					// Built-in
#include <math.h>					// Built-in


/*
 *	If the 'ELEVATION' symbol is set to 'false' all of the elevation control functions
 *	are disabled. This is for use with the azimuth only rotators such as the SPID-RAU
 *	or SPID-RAK.
 */

#define	ELEVATION	true			// Compile for EL/AZ rotator


/*
 *	Some ofthe PC based programs that can interface with the controller may have
 *	baud rate limitations, so check the documentation for the program(s) you might
 *	be using. Most allow you to specify a baud rate up to some maximum; the one exception
 *	is the N1MM+ Rotor program in which the baud rate is fixed at 9600.
 */

	#define	BAUD_RATE		9600	// Baud rate for the USB port (for N1MM++)


/*
 *	The EEPROM contents on a brand new processor are undefined (although usually 0x00 
 *	or 0xFF), so we define an arbitrary number that will allow the program to determine
 *	if the previously saved azimuth and elevation are valid.
 */

	#define EEPROM_VALID	12345	// Should be in the EEPROM if previously used


/*
 *	'MOTOR_TIMEOUT' is used to detect that one of the motors has hit the mechanical
 *	endstop. The manual says the motors take two minutes for 360 degrees of movement.
 *	In testing, my rotator takes about 2 1/2 minutes to turn 360 degrees.
 *
 *	In my test setup, the pulses are occurring approximately every 410 milliseconds
 *	with the 12V power supply and about every 340 milliseconds with the 15V power supply.
 *	Settin the the timeout for 700 milliseconds seems to work fine in either case in
 *	testing.
 */

	#define	MOTOR_TIMEOUT	 700UL	// Might need tweaked based on cable length


/*
 *	These define the movement limits. The azimuth has no endstops, so the limits are
 *	arbitrary. The software currently has no provision to handle negative azimuths
 *	so we limit the azimuth travel to 360 degrees.
 *
 *	The elevation motor does have endstops and can do a bit more than 180 degrees, but
 *	I only need 90 for now.
 *
 *	Perhaps in the future, I will add code to handle azimuth changes outside the current
 *	limits and allow the elevation rotator to move 180 degrees. Allowing the elevation
 *	motor to move 180 degrees would also require logic to change the azimuth reading
 *	when the elevation is greater than 90 degrees.
 */

	#define	MAX_AZIMUTH		360				// Maximum azimuth allowed
	#define	MIN_AZIMUTH		  0				// Minimum azimuth allowed

	#define	MAX_ELEVATION	 90				// Maximum elevation allowed
	#define	MIN_ELEVATION	  0				// Minimum elevation allowed


/*
 *	'CAL_ADJUSTMENT' is the number of milliseconds to back up the elevation motor when
 *	it hits one of the mechanical end stops. By backing it off a tad, we eliminate any
 *	mechanical stress on the mechanism.
 */

	#define	CAL_ADJUSTMENT		50UL		// Try 50 milliseconds for starters


/*
 *	The 'DEBOUNCE' time determines how long we wait after seeing the start of a pulse
 *	from one of the motors before we process it.
 */

	#define	DEBOUNCE		 10UL			// Debounce time (10 milliseconds)


/*
 *	These symbols define the azimuth and elevation positions for the 'Park' command. I
 *	chose to park the elevation motor at zero degrees (level) and the azimuth motor at
 *	42 degrees. The choice for the azimuth position in my case allows me to visually
 *	line the antenna up with the roof ridgeline of my house.
 *
 *	You can optionally elect to park only one or the other, or even neither! If, for
 *	example you only want to park the elevation, comment out the definition for
 *	'AZ_PARK'. If you don't want to park either, comment them both out.
 */

 	#define	AZ_PARK		42
 	#define	EL_PARK		 0
 

/*
 * 'EPROM_TIMEOUT' defines the number of milliseconds to wait after the rotator stops moving to
 * update the saved azimuth and elevation in the EEPROM. It is currently set to 10 seconds.
 */

	#define	EEPROM_TIMEOUT	10000			// 10 seconds in milliseconds


/*
 *	'BTN_READ_TIME' defines how often we look at one of the push buttons to see if it
 *	is still pushed. The time needs to be less than the interval between motor
 *	pulses or the motors will constantly stop and start, which is hard on the relays.
 *	
 *	It should be high enough high enough to allow the operator to see the changes
 *	in the targets.
 */

	#define BTN_READ_TIME	200UL			// Seems to be a good value


/*
 *	The following definitions control the button accelerator. If 'BTN_FAST' is
 *	set to true, and a button is hels for more than 'BTN_FAST_TIME', the target
 *	angle will be incremented or decremented by 'BTN_FAST_INCR' degrees instead
 *	of the default one degree.
 */

	#define	BTN_FAST		true			// If 'true' change angle by
	#define BTN_FAST_INCR	5				// this many degrees
	#define	BTN_FAST_TIME	2000UL			// if button held longer than this (2 seconds)

#endif										// #if SPID_H	
