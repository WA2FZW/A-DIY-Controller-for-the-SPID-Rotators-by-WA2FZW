/*
 *	SPID Rotator Controller - Version 1.2 - 08/15/2024
 *
 *		This software was originally designed to control the SPID-RAS Elevation/Azimuth
 *		rotator, but has been since modified to support the azimuth only SPID rotators
 *		such as the SPID-RAU and SPID-RAK.
 *
 *		This rotator works similarly to the AR-22 which I designed a controller for
 *		several years ago in that it reports positions via a switch closure in the
 *		motor units. Where the AR-22 sent a switch closure every 6 degrees (give or
 *		take), the SPID rotators send pulses every one degree. That combined with the
 *		fact that it also uses DC motors makes it very accurate.
 *
 *		The program is designed work with a number of PC based programs that are
 *		capable of sending azimuth and elevation commands. We use a subset of the
 *		Yaesu GS-232 language (plus a couple of commands that I invented for testing
 *		and debugging).
 *
 *		The programs I generally use are the N1MM+ Rotator program, SatPC32 and DxView
 *		to name a few.
 *
 *		Version 1.1:
 *
 *			Added the 'ELEVATION' symbol, which when 'false' disables all of the elevation
 *			control functions; i.e. for the SPID-RAU or SPID-RAK rotators which are azimuth
 *			only rotators.
 *
 *		Version 1.2:
 *
 *			I completely overhauled the calibration procedure. Calibration can no longer
 *			be started via a command from the USB port; only by pressing and releasing the
 *			'CAL' button. The procedure is explained in detail in the documentation.
 *
 *			Eliminated the use of interrupts to check for the index pulses. It just wasn't
 *			working!
 *
 *			Several minor changes in various parts of the code; explained where the 
 *			changes were made.
 */

/*
 *	Setting "DEBUG" to "true" will turn on a lot of print statements that show you
 *	what the program is thinking. It should only be enabled if you are using the
 *	Arduino IDE's serial monitor (or some other terminal emulator) to talk to it as
 *	the debug outputs confuse other programs such as N1MM+ Rotor or SatPC32.
 */

#define	DEBUG	false				// Enables verbose output when set to "true"

#include "SPID.h"					// Definitions that might have to be tweaked.


/*
 *	Define the pin assignments and other stuff for the motors. The pin names match
 *	or are similar to the 'NET' names on the schematic.
 *
 *	The list is in GPIO pin number order. The order might seem a bit strange. Other
 *	than pins that used to need to be interrupt capable and some that are fixed such
 *	as those for the display, the assignments were chosen to facilitate the PCB layout.
 */

#define SELECT			 4			// Operates the relay that selects which motor unit to run
#define	PWR_SENSE		13			// Senses whether main power supply is turned on
#define DIRECTION		14			// Operates the relay that selects the direction of rotation
#define	EL_INDEX		18			// Position impulses from the elevation motor
#define	AZ_INDEX		19			// Position impulses from the azimuth motor
#define	LCD_SDA			21			// Display data line
#define	LCD_SCL			22			// Display clock line

#define	REBOOT			A1			// Initiates a software reboot when grounded
#define	OPERATE			A3			// Operates the relay to apply power to the selected motor unit
#define	SAVED_LED		A7			// Elevation & Azimuth saved LED
#define	CW_BTN			A8			// Pushbutton to initiate clockwise (azimuth) rotation
#define	CAL_BTN			A9			// Pushbutton to initiate a system calibration
#define	DOWN_BTN	   A10			// Pushbutton to initiate down (elevation) rotation
#define	UP_BTN		   A11			// Pushbutton to initiate up (elevation) rotation
#define	CCW_BTN		   A12			// Pushbutton to initiate counterclockwise (azimuth) rotation

#define	NO_BTN			 0			// Not a pin number but indicates no button is active


/*
 *	Define the parameters for the LCD display.
 *
 *	The height and width parameters should not be changed, however, I've run across
 *	some displays that have an address of 0x3F. If the display doesn't work, There
 *	is a program available on GitHub (https://gist.github.com/tfeldmann/5411375)
 *	that will scan for I2C devices and report the addresses of any found, if it finds
 *	the display is at a different address, change it here.
 */

#define LCD_HEIGHT		 4					// Display is 4 Lines high
#define LCD_WIDTH		20					// and 20 characters wide
#define	LCD_ADDRESS	  0x27					// Display's I2C address


/*
 *	The following definitions provide more easily understandable names for the relay
 *	states needed to run the motor units.
 *
 *	Rather than two sets of relays for each motor, I chose to only operate one of them
 *	at a time.
 *
 *	Relay K3 on the PCB selects either the azimuth or the elevation motor; relay
 *	K2 selects the direction to turn the motor. Once those 2 relays are set properly, relay
 *	K1 is operated  after a slight delay to apply power to the selected motor.
 *
 *	There are two reasons I did it this way:
 *
 *		1.	It reduces the current requirement on the motor power supply since only one
 *			motor will be running at any given time. The power supply is only capable of
 *			providing 3 amps and the motors require 2 amps.
 *
 *		2.	It simplifies the software if I only have to keep track of one position at
 *			a time.
 *
 *	The 'TURN_CW' and 'TURN_CCW' definitions for the azimuth motor have been confirmed.
 *	The 'MOVE_UP' and 'MOVE_DOWN' definitions will have to be confirmed based on how
 *	the antenna is mechanically mounted in the elevation unit.
 *
 *	If one of the motors runs in the wrong direction, the 'HIGH' and 'LOW' assignments
 *	can be flip-flopped.
 */

#define	SELECT_EL	HIGH				// HIGH on K3 selects the elevation motor
#define	SELECT_AZ	LOW					// LOW on K3 delects the azimuth motor

#define	TURN_CCW	HIGH				// Rotate clockwise (azimuth motor)
#define	TURN_CW		LOW					// Rotate counter-clockwise (azimuth motor)

#define	MOVE_UP		HIGH				// Increase elevation
#define	MOVE_DOWN	LOW					// Decrease elevation

#define	RUN_MOTOR	LOW					// Operate the relay to power the selected motor
#define	STOP_MOTOR	HIGH				// Turn the motor off

#define	OFF			HIGH				// HIGH on any of the relays turns them off


/*
 *	The above definitions apply solely to the GPIO pin states required to set
 *	the relays correctly. The following definitions are used by the software
 *	to make decisions regarding which motors are running and in what direction
 *	the motors are actually turning.
 */

#define	MTR_OFF		0						// No movement
#define	MTR_CW		1						// Run azimuth CW
#define	MTR_CCW		2						// Run azimuth CCW
#define	MTR_UP		4						// Run elevation UP
#define	MTR_DOWN	8						// Run elevation DOWN

#define	MTR_AZ		( MTR_CW | MTR_CCW )	// These are macros essentially, so
#define	MTR_EL		( MTR_UP | MTR_DOWN )	// the '()' are required


/*
 *	These definitions define symbols for the various reasons for stopping the motor(s).
 *	They also are incidies into the "stopReasons' array of Strings which will be displayed
 *	when debugging is enabled.
 */

#define	NO_REASON	0
#define	EL_TIMEOUT	1
#define	AZ_TIMEOUT	2
#define	EL_NORMAL	3
#define	AZ_NORMAL	4
#define	EL_CAL		5
#define	EL_BACKUP	6
#define	REVERSAL	7
#define	SW_REBOOT	8


/*
 *	These are the Strings that will be displayed when a motor is stopped and debugging
 *	is enabled:
 */

String stopReasons[] = 
	{ "Unknown",            "Elevation Timeout",  "Azimuth Timeout",
	  "EL - Normal Stop",   "AZ - Normal Stop",   "Elevation Calibration",
	  "Elevation Backup",   "Reversal",            "Reboot" };


/*
 *	These two variables are used throughout the program to remember which (if either)
 *	motor is running and which direction it is moving.
 */

uint8_t activeMotor = MTR_OFF;				// Active motor
uint8_t	dirRelay    = OFF;					// Direction relay setting


/*
 *	These are for the 'SAVED_LED' status:
 */

#define	LED_ON	LOW
#define	LED_OFF	HIGH


/*
 *	Define some needed global variables:
 */

uint32_t	buttonReadTime = 0;				// Last time we looked at the pushbuttons
uint8_t		activeButton   = NO_BTN;		// We never looked at the pushbuttons
uint32_t	buttonActiveTime = 0;			// When the active button went active

uint32_t	azStopTime;						// Last time the azimuth motor stopped
uint32_t	elStopTime;						// Last time the elevation motor stopped

volatile uint32_t	pulseTime = 0;			// Time of last index pulse for timeout function


/*
 *	When nothing is happening, the first line of the display shows the name of the program,
 *	"SPID Controller". When things are happening (like calibration for example), the first
 *	line of the display will have other information such as "** AZ Calibrating **". We leave
 *	the useful information on the display for 'helloTimeout' milliseconds after it is no
 *	longer useful before re-displaying the default text.
 */

uint32_t	notHelloTime   = 0;				// Last time something other than hello displayed
uint32_t	helloTime      = 0;				// Current time
uint32_t	helloTimeout   = 3000;			// 3 second refresh delay
bool		helloMsg       = false;			// True when the default message is on line 1 of the LCD


/*
 *	These define the initial state of things:
 */

volatile bool	azIsCalibrating = false;	// True when azimuth rotator is calibrating
volatile bool	elIsCalibrating = false;	// True when elevation rotator is calibrating

volatile bool	tgtAzChanged   = false;		// True when 'tgtAz' has changed
volatile bool	tgtElChanged   = false;		// True when 'tgtEl' has changed

volatile bool	curAzChanged   = false;		// True when 'curAz' has changed
volatile bool	curElChanged   = false;		// True when 'curEl' has changed

bool			saveAz = false;				// 'true' when azimuth needs to be saved
bool			saveEl = false;				// 'true' when elevation needs to be saved


/*
 *	The 'tgtAz', 'curAz', 'tgtEl' and 'curEl" variables are used to display the values
 *	on the display and to send the current values to the USB output when requested.
 */

int16_t	tgtAz = 0;				// Target azimuth
int16_t	curAz = 0;				// Current azimuth

int16_t	tgtEl = 0;				// Target elevation
int16_t	curEl = 0;				// Current elevation

int16_t	lastGoodAz = 0;			// Last good azimuth reading
int16_t	lastGoodEl = 0;			// Last good elevation reading


/*
 *	We periodically (currently 10 seconds) after the last movement of one of the motors
 *	save the current elevation and/or current azimuth in the EEPROM. Here we define the
 *	addresses for those entries.
 *
 *	All the entries in the EEPROM are 2 bytes.
 */

#define	VALIDATION_ADDRESS	0				// EEPROM address of the validation code
#define	AZ_ADDRESS			2				// EEPROM Address for stored 'curAz'
#define	EL_ADDRESS			4				// EPROM Address for stored 'curEl'


/*
 *	The power supply PCB has a provision to tell the processor whether or not it is turned
 *	on. When the controller is connected to a PC, the processor remains powered up and is
 *	capable of processing some commands, but if the main power supply is not on, it will
 *	be incapable of actually executing commands that initiate movement of either motor.
 *
 *	If the power supply is not on, we disable the pushbuttons and turn the display 
 *	off. Commands received from the PC that would initiate a change of elevation or
 *	azimuth are also ignored. Commands such as 'C' or 'C2' can still be answered even if
 *	the power is off.
 */

volatile bool	powerStatus = true;			// Assume power supply is on
volatile bool	powerWasOff = true;			// But was it previously off?


/*
 *	These buffers are used to queue stuff to be displayed on the LCD. The approach to
 *	displaying things on the LCD is a leftover from the original AR-22 code. The AR-22
 *	controller used an encoder to change the azimuth and trying to update the display
 *	from within the encoder ISR caused system lockups.
 *
 *	Use of the 'String' class makes constructing dynamic strings easier than if
 *	we had used 'char' arrays.
 */

String	lcdBuffer_1;						// Line 1 buffer
String	lcdBuffer_2;						// Line 2 buffer
String	lcdBuffer_3;						// Line 3 buffer
String	lcdBuffer_4;						// Line 4 buffer


/*
 *	Define most of the various strings to be displayed on the LCD (some are built
 *	piecemeal in the code), but these are the fixed ones. The number of characters
 *	in each is based on the assumption that a 20 character by 4 line display is
 *	being used.
 *
 *	The Strings include trailing blanks. If those were not there, the remnants of
 *	previously displayed messages would remain on the screen.
 */

#define	helloString			"  SPID Controller   "		// Normal first line of display

#if ( ELEVATION )
#define versionString		" Version 1.2 AZ/EL  "		// Curent software version
#else
#define versionString		"Version 1.2 AZ Only "		// Curent software version
#endif

#define	headerString		"           Cur  Tgt "		// Header on line 2
#define	azHeader			"Azimuth:            "		// 3rd line heading
#define	elHeader			"Elevation:          "		// 4th line heading
#define	elTimeoutString		" *** EL Timeout *** "		// Elevation motor timeout
#define	azTimeoutString		" *** AZ Timeout *** "		// Azimuth motor timeout
#define	azCalibrateString	"** AZ Calibrating **"		// Line 1 when calibration in progress
#define	elCalibrateString	"** EL Calibrating **"		// Line 1 when calibration in progress
#define	azCalibratedString	"** AZ Calibrated ** "		// Line 1 when calibration is finished
#define	elCalibratedString	"** EL Calibrated ** "		// Line 1 when calibration is finished
#define blankString			"                    "		// Used to clear the display


/*
 *	The 'workBuffer' is used by several functions as a scratchpad in which to build
 *	various output messages, thus we define it globally.
 */

String	workBuffer;							// General use buffer


/*
 *	Create the display object. The URL for the library can be found in the 'SPID.h'
 *	header file. There are a number of different LCD libraries. The one specified in the
 *	header file works with the ones I have. If using  a different display, you might need
 *	a different library and possibly minor changes in the software as to how it is
 *	initialized.
 */

LiquidCrystal_I2C lcdDisplay ( LCD_ADDRESS, LCD_WIDTH, LCD_HEIGHT );


/*
 *	The 'setup' function is called by the Arduino boot loader when the controller
 *	is first powered up, or after a reset. Here, we initialize anything that needs
 *	to be set up before we really go to work.
 */

void setup ()
{
uint16_t	validEEPROM;					// Used to validate EEPROM contents

	Serial.begin ( BAUD_RATE );				// See comments in 'SPID.h' and the documentation
	delay ( 1000 );							// Blindly wait 1 second for connection


/*
 *	Setup the I/O pins that control the motor relays:
 */

	pinMode ( DIRECTION, OUTPUT ); 			// Direction selector relay
	pinMode ( SELECT,    OUTPUT ); 			// Motor selection relay
	pinMode ( OPERATE,   OUTPUT ); 			// Motor run relay

	digitalWrite ( OPERATE,   OFF );		// If a motor started in run mode, turn it off
	delay ( 10 );							// To avoid switching the other relays under power
	digitalWrite ( DIRECTION, OFF );		// Turn the 'DIRECTION' relay off
	digitalWrite ( SELECT,    OFF );		// and the motor 'SELECT' relay

	activeMotor = MTR_OFF;					// No motor running
	dirRelay    = OFF;						// Default direction


/*
 *	The 'PWR_SENSE' is the indication that the main power supply is on or off:
 */

	pinMode ( PWR_SENSE, INPUT );			// Power supply on/off indicator


/*
 *	Initialize the display.
 */

	lcdDisplay.init ();							// Initialize the LCD
	lcdDisplay.setBacklight ( true );			// Turn the light on

	LCDPrint ( 1, String ( helloString ));		// We're almost on the air!
	LCDPrint ( 2, String ( versionString ));

	delay ( 2000 );								// Time to read the splash screen


/*
 *	The original AR-22 controller used an encoder to set the target azimuth. Rather than
 *	use 2 encoders for azimuth and elevation, I elected to use pushbuttons for this version.
 *
 *	The switch panels do not have pullup resistors, so we use the ones built into the
 *	Arduino. Note that should I decide to switch the processor to an ESP32, pullups will
 *	have to be added to the switch panels as the ESP32 does not have the internal ones.
 */

	pinMode ( UP_BTN,   INPUT_PULLUP );			// Elevation up button
	pinMode ( DOWN_BTN, INPUT_PULLUP );			// Elevation down button
	pinMode ( CW_BTN,   INPUT_PULLUP );			// Azimuth clockwise  button
	pinMode ( CCW_BTN,  INPUT_PULLUP );			// Azimuth counterclockwise button
	pinMode ( CAL_BTN,  INPUT_PULLUP );			// Calibrate button


/*
 *	The 'EL_INDEX' and 'AZ_INDEX' pins generate pulses for every degree of movement
 *	on the respective motors. These pins don't need pullup resistors. The pins are
 *	normally 'HIGH'. They go 'LOW' when a pulse is occurring.
 */

	pinMode ( AZ_INDEX, INPUT );
	pinMode ( EL_INDEX, INPUT );


/*
 *	Setup the 'SAVED_LED' GPIO pin and the 'REBOOT" pin and turn the LED on:
 */

	pinMode ( SAVED_LED , OUTPUT );			// Assign the GPIO pin
	digitalWrite ( SAVED_LED, LED_ON );		// And turn it on

	pinMode ( REBOOT, INPUT_PULLUP );		// Software reboot button


/*
 *	Set some initial conditions:
 */

	azIsCalibrating	 = false;				// Azimuth motor is not being calibrated
	elIsCalibrating	 = false;				// Neither is the elevation motor

	tgtAzChanged	 = false;				// 'tgtAz' hasn't changed
	curAzChanged	 = false;				// 'curAz' hasn't changed
	tgtElChanged	 = false;				// Neither has 'tgtEl'
	curElChanged	 = false;				// or 'curEl'


/*
 *	The 'curAz' and 'curEl' from the last time the controller was used should be stored
 *	in the EEPROM. The only exception would be the first time the controller is turned
 *	on with a new processor.
 *
 *	When the controller is turned on for the first time, there could be anything in the
 *	EEPROM. Generally, one would expect to find either 0x00 or 0xFF in all locations in
 *	the EEPROM if it has never been used.
 *
 *	It's not a sure thing, but every time the controller is started, we will look for
 *	a code in the EEPROM that is highly unlikely to be randomly present, and if we don't
 *	get the right answer for that, we assume the EEPROM has never been used.
 *
 *	If the validation is successful, we get the last recorded azimuth and elevation from
 *	the appropriate EEPROM addresses. If unsuccessful, we write the correct validation
 *	code into the EEPROM, perform a calibration sequence which will set the calibrated
 *	current azimuth and elevation to the EEPROM and in the corresponding variables.
 *
 *	If you un-comment the following line of code, it will test the EEPROM validation
 *	process by putting an invalid code in the 'VALIDATION_ADDRESS'.
 */

// 	EEPROM.put ( VALIDATION_ADDRESS, 0 );

	EEPROM.get ( VALIDATION_ADDRESS, validEEPROM );	// Code that says EEPROM contents valid

	if ( validEEPROM == EEPROM_VALID )				// Validation correct?
	{
		EEPROM.get ( AZ_ADDRESS, curAz );			// Yes, get the current azimuth
		EEPROM.get ( EL_ADDRESS, curEl );			// and current elevation

		lastGoodAz = tgtAz = curAz;					// Set targets same as current values
		lastGoodEl = tgtEl = curEl;
	}

	else											// Validation was not correct
	{
		validEEPROM = EEPROM_VALID;
		EEPROM.put ( VALIDATION_ADDRESS, validEEPROM );	// Correct validation code into the EEPROM
		Calibrate ();									// Run the calibration sequence
	}


/*
 *	We already put the default startup message on the display, but here we send it
 *	to the USB port only if 'DEBUG' is enabled. If we're not debugging, the message
 *	could cause confusion to any of the rotator control programs we might be using
 *	on the PC.
 */

	if ( DEBUG )
	{
		Serial.print ( helloString );
		Serial.println ( versionString );
	}

	SayHello ();							// Display the program & version plus the fixed headers
}											// End of 'setup'


/*
 *	Once the 'setup' function has completed, the 'loop' function runs forever or at
 *	least until the power is turned off or the hardware is reset.
 */

void loop ()
{

/*
 *	Each time through the loop we check the status of the main power supply. If
 *	it is on, the 'PWR_SENSE' will show a 'HIGH'; 'LOW' if it is off. 
 *
 *	If it is now off, but was previously on, we turn off the backlight and erase
 *	the display to prevent pixels from being 'burned in'.
 *
 *	If it was previously off but is now on, we turn the basklight on and restore
 *	the data on the display.
 */

	powerStatus = digitalRead ( PWR_SENSE );		// Check power supply on/off status

	if ( !powerStatus && !powerWasOff )				// Was on, but now off
	{
		powerWasOff = true;							// so previously off
		
		lcdDisplay.setBacklight ( false );			// Turn the display backlight off

		for ( int i = 1; i < 5; i++ )
			LCDPrint ( i, String ( blankString ));	// Erase the display
	}
	
	if ( powerStatus && powerWasOff )				// Was off but on now
	{
		powerWasOff = false;

		lcdDisplay.setBacklight ( true );			// Turn the display backlight on

		LCDPrint ( 1, helloString );				// We're on the air!
		LCDPrint ( 2, headerString );				// Display header fields
		LCDPrint ( 3, azHeader );
		if ( ELEVATION )
			LCDPrint ( 4, elHeader);				// Only if elevation type rotator

		tgtAzChanged   = true;						// Force update of all the numbers
		tgtElChanged   = true;
		curAzChanged   = true;
		curElChanged   = true;

		UpdateNumbers ();
	}


/*
 *	First, we need to check to see if the operator is trying to initiate a change of
 *	the azimuth or elevation using the manual pushbuttons.
 *
 *	Also, when a button is active, USB commands which would change the targets
 *	are disabled; i.e., the buttons have priority over the PC commands. Only
 *	commands arriving on the USB port that don't change the target azimuth or
 *	elevation are processed when a button is active.
 */

	if ( millis () - buttonReadTime > BTN_READ_TIME )	// Time to check them?
		CheckButtons ();								// Yep - look for manual control


/*
 *	The next thing we need to do is see if there is an ASCII command to be processed on
 *	the USB port. We need to do this regardless of whether the power supply is on or
 *	not as one of the PC programs may be asking for a position report and they might
 *	get upset if the don't get an answer in a timely manner.
 *
 *	Note that the 'GetCommand' function contains code to disable any commands that
 *	would try to initiate operation of one of the motors if the power supply is
 *	turned off and will completely handle any status requests internally.
 */

	GetCommand ();


/*
 *	Next, we see if we need to start one of the motors. The hardware only allows one
 *	or the other to be operating at any given time. For EME work, neither ever has to
 *	turn more than a degree or two. For satellite work, the elevation is more likely to
 *	change particularly when a satellite passes 360 degrees and a complete turn of
 *	the antenna is required, thus we give the elevation motor priority.
 *
 *	Perhaps someday I'll figure out how to time slice them!
 *
 *	We only check to see if one or the other needs to start if neither one is already
 *	running.
 */

	if ( !activeMotor )								// If neither motor running
	{
		if ( ELEVATION )							// If not an azimuth only rotator
		{
			if ( tgtEl > curEl )					// Need to raise the antenna?
				StartMotor ( MTR_EL, MOVE_UP );		// Yes - Do it!

			if ( tgtEl < curEl )					// Need to lower the antenna?
				StartMotor ( MTR_EL, MOVE_DOWN );	// Yes - Do it!
		}
	}


/*
 *	If we didn't start the elevation motor, the 'activeMotor' will still show 'MTR_OFF'
 *	and if that is the case, we check to see if we need to start the azimuth motor.
 */

	if ( !activeMotor )								// If neither motor running
	{
		if ( tgtAz > curAz )						// Need to turn clockwise?
			StartMotor ( MTR_AZ, TURN_CW );			// Yes - Do it!

		if ( tgtAz < curAz )						// Need counterclockwise rotation?
			StartMotor ( MTR_AZ, TURN_CCW );		// Yes - Do it!
	}


/*
 *	At this point, one of the motors may or may not be running which will be indicated
 *	by the value of the 'activeMotor' variable. If one is running, we need to check a
 *	few conditions (I may have to play with the order here):
 *
 *		o  Did we get an index pulse?
 *		o  Did a timeout occur?
 *		o  Did a command come in that would cause us to have to reverse direction?
 *
 *	Each of these functions also has a test to see if a motor is actually running as
 *	each of the functions could cause the active motor to stop.
 */

	if ( activeMotor )						// If nothing running - No need to check these
	{
		CheckPulse ();						// If we got an index pulse, process it
		CheckTimeout ();					// Did a timeout occur?
		CheckReversal ();					// Need to reverse a motor?
	}


/*
 *	Prior to Version 1.2, a one second timer interrupt was used to invoke the
 *	'UpdateNumbers' function. In Version 1.2, that was eliminated and now we
 *	check to see if anything actually needs to be updated on the screen.
 */

	if ( tgtAzChanged | curAzChanged | tgtElChanged | curElChanged )
		UpdateNumbers ();


/*
 *	If it's been more than 'EEPROM_TIMEOUT' seconds since the current azimuth was
 *	last saved and the 'saveAz' flag is 'true' we save the current azimuth in the
 *	EEPROM and clear the 'saveAz' flag. Then we do the same for the elevation.
 */

	if ((( millis() - azStopTime ) >= EEPROM_TIMEOUT ) && saveAz )
	{
		EEPROM.put ( AZ_ADDRESS, curAz );				// Saved
		saveAz = false;									// Clear the flag
	}

	if ( ELEVATION )
		if ((( millis() - elStopTime ) >= EEPROM_TIMEOUT ) && saveEl )
		{
			EEPROM.put ( EL_ADDRESS, curEl );			// Saved
			saveEl = false;								// Clear the flag
		}


/*
 *	If both flags are clear, we turn the 'SAVED_LED' on. If one or the other still
 *	needs to be saved, we turn the LED off. If a motor is running, we don't save
 *	the numbers.
 */

	if ( !activeMotor )							
	{
		if ( saveAz || saveEl )							// If either need saved,
			digitalWrite ( SAVED_LED, LED_OFF );		// turn the LED off

		else											// Otherwise,
			digitalWrite ( SAVED_LED, LED_ON );			// turn the LED on
	}


	SayHello ();							// Display the program & version plus the fixed headers

/*
 *	Finally, we see if the operator has pushed the 'REBOOT' button, in which case
 *	we need to save the current azimuth and elevation and restart the program.
 */

	CheckReboot ();
}														// End of 'loop'


/*
 *	'StartMotor' is called when we need to start one of the rotator motors.
 *
 *	The first argument specifies which one ('MTR_AZ' or 'MTR_EL'). We only allow
 *	one motor to be running at a time, so if one is already running we don't do
 *	anything.
 *
 *	The second argument specifies the proper setting for the direction relay.
 *
 *	We can't start either one if the main power supply is turned off.
 */

void StartMotor ( uint8_t whichOne, uint8_t dir )
{
	if ( activeMotor || !powerStatus )		// Power off or already running?
		return;								// Can't do anything


/*
 *	Ok, neither motor is running, so let's see which one needs to be started. For either
 *	EME operations, neither the azimuth or elevation changes dramatically. For satellite
 *	operations, the azimuth is more likely to change more frequently, particlarly when
 *	the satellite passes North and we have to do a full rotation, so azimuth gets priority.
 */

	if ( whichOne == MTR_AZ )							// Request for azimuth change?
	{
		if ( DEBUG )
		{
			Serial.print ( "Start Azimuth Motor - " );	// Let the operator know if watching

			if ( dir == TURN_CW )						// the serial monitor.
				Serial.println ( "CW, " );

			else
				Serial.println ( "CCW," );
		}

		activeMotor = whichOne;						// Remember which motor is running
		dirRelay = dir;								// And remember direction

		digitalWrite ( SAVED_LED, LED_OFF );		// Azimuth no longer saved
		saveAz = true;								// Will need to be saved later

		azStopTime = pulseTime = millis ();			// Pretend we saw a pulse
		
		digitalWrite ( SELECT, SELECT_AZ );			// Select the azimuth motor
		digitalWrite ( DIRECTION, dir );			// and set the direction relay
		delay ( 10 );								// Let those settle before applying power
		digitalWrite ( OPERATE, RUN_MOTOR );		// Start it up!
	}												// End of 'if ( whichOne == MTR_AZ )'

	if  ( ELEVATION && ( whichOne == MTR_EL ))		// Request for elevation change?
	{
		if ( DEBUG )
		{
			Serial.print ( "Start Elevation Motor - " );

			if ( dir == MOVE_UP )
				Serial.println ( "UP, " );

			else
				Serial.println ( "DOWN," );
		}

		activeMotor = whichOne;					// Remember which motor is running
		dirRelay = dir;							// And remember direction

		digitalWrite ( SAVED_LED, LED_OFF );	// Elevation no longer saved
		saveEl = true;							// Need to save later

		elStopTime = pulseTime = millis ();		// Pretend we saw a pulse

		digitalWrite ( SELECT, SELECT_EL );		// Select the elevation motor
		digitalWrite ( DIRECTION, dir );		// and set the direction relay
		delay ( 20 );							// Let those settle before applying power
		digitalWrite ( OPERATE, RUN_MOTOR );	// Start it up!

	}											// End of 'if ( whichOne == MTR_EL )'
}												// End of 'StartMotor'


/*
 *	'StopMotors' turns the rotator motors off. Because of the hardware configuration,
 *	only one motor can really be running.
 *
 *	Again, there is a 10mS delay between turning the power relay off and turning the
 *	direction and selection relays off to make sure the power is really off before
 *	operating those relays.
 *
 *	The 'why' argument is for debugging. If the 'DEBUG' symbol is turned on, the function
 *	displays the reason the motor is being stopped.
 *
 *	Modified in Version 1.2:
 *
 *		Instead of the 'why' argument being a 'String' it is now an index into the
 *		'stopReasons' array of strings.
 *
 *		If there is an 'activeButton', sometimes we don't stop the motor. We do stop them
 *		if the reason is a timeout.
 */

void StopMotors ( uint16_t why )
{
	if ( activeButton )							// Don't stop if a button is active
	{
		switch ( why )							// 'Switch' makes the list easier to modify
		{										// Than using 'if' statements
			case EL_NORMAL:
			case AZ_NORMAL:
				return;							// Eliminates relay chatter
		}
	}

	if ( activeMotor == MTR_AZ )				// Was the azimuth motor running?
	{
		azStopTime = millis ();					// Record actual stop time
		saveAz = true;							// Azimuth will need to be saved
	}

	if ( ELEVATION )							// Only if not azimuth only rotator
		if ( activeMotor == MTR_EL )			// Or was it the elevation motor?
		{
			elStopTime = millis ();				// Record actual stop time
			saveEl = true;						// Elevation will need to be saved
		}

	activeMotor = MTR_OFF;						// Neither motor running now

	digitalWrite ( OPERATE, STOP_MOTOR );		// Motor power off!
	delay ( 10 );								// Don't change the other relays under power
	digitalWrite ( SELECT, OFF );				// Turn the selection relay off
	digitalWrite ( DIRECTION, OFF );			// and the direction relay

	azIsCalibrating = false;					// and neither motor
	elIsCalibrating = false;					// is being calibrated

	if ( DEBUG )
	{
		Serial.print ( "StopMotors - " );
		Serial.print ( stopReasons[why] );
		Serial.print ( " - tgtAz = " );		Serial.print ( tgtAz );
		if ( ELEVATION )
			Serial.print ( ",  tgtEl = " );	Serial.print ( tgtEl );
		Serial.print ( '\n' );
	}

	UpdateNumbers ();							// Update the display
}


/*
 *	'CheckTimeout' checks to see if one of the rotator motors has timed out. That happens
 *	if one of the motors is running and we didn't see an index pulse within the time
 *	limit set in the 'MOTOR_TIMEOUT' symbol.
 *
 *	The SPID-RAS has no mechanical endstops on the azimuth rotator, nor do the azimuth
 *	only rotators. There are endstops on the elevation rotator. If a timeout occurs on
 *	the azimuth rotator, we have to assume there is some problem and we stop it
 *	immediately and assume the current azimuth is correct. We will also set the target
 *	azimuth equal to the current azimuth.
 *
 *	If we get a timeout on the elevation rotator we have to handle it a bit differently.
 *	As the 'MAX_ELEVATION' symbol is set to way less than the upward endstop, if the motor
 *	was moving up, we stop the motor and set the target and current elevations to the
 *	last good elevation. If it was moving down, we assume it hit the lower endstop and
 *	set the target and current elevations to 'MIN_ELEVATION'; that may or may not actually
 *	be the case.
 */

bool CheckTimeout ()
{
	uint8_t oldMotor = activeMotor;					// These get cleared by 'StopMotor'
	uint8_t oldRelay = dirRelay;					// so we need to remember them

	if ( !activeMotor )								// Nothing running?
		return false;								// Nothing to check here

	if (( millis () - pulseTime ) <= MOTOR_TIMEOUT )	// Out of time?
		return false;									// Nope!

	if ( ELEVATION &&  ( oldMotor == MTR_EL ))		// Elevation motor running
	{
		StopMotors ( EL_TIMEOUT );					// Turn both motors off

		StartMotor ( MTR_EL, !oldRelay );			// Start it in the opposite direction
		delay ( CAL_ADJUSTMENT );					// Time based
		StopMotors ( EL_BACKUP );					// Stop them again

		if ( oldRelay == MTR_UP )					// Was moving up,
			tgtEl = curEl = lastGoodEl;				// so maximum elevation

		else										// If moving down we assume it hit the
			tgtEl = curEl = MIN_ELEVATION;			// endstop so set to minimum elevation

		EEPROM.put ( EL_ADDRESS, curEl );			// Write new elevation
		saveEl = false;								// It's saved
		lastGoodEl = curEl;							// It's now the last good one

		tgtElChanged = true;						// Signal to update the display

		if ( !elIsCalibrating )
		{
			lcdBuffer_1 = elTimeoutString;
			notHelloTime = millis ();
			CheckLCD ();							// Force display
		}
	}

	if ( oldMotor == MTR_AZ )						// Azimuth motor running
	{
		StopMotors ( AZ_TIMEOUT );					// Turn both motors off

		tgtAz = curAz = lastGoodAz;					// Adjust azimuth

		EEPROM.put ( AZ_ADDRESS, curAz );			// Write new azimuth
		saveAz = false;								// It's saved

		lcdBuffer_1 = azTimeoutString;
		notHelloTime = millis ();
		CheckLCD ();								// Force display

		tgtAzChanged = true;						// Signal to update the display
	}

	UpdateNumbers ();								// Display changed numbers


/*
 *	To make the timeout more obvious, we're going to blink the saved LED a few times.
 */

	BlinkLED ();									// Inform operator


/*
 *	If one motor timed out, we saved the azimuth or elevation appropriately, but the
 *	other motor's current setting might still need to be saved, so we check to see if
 *	the LED needs to be on or off.
 */

	if ( saveAz || saveEl )							// If either needs saved,
		digitalWrite ( SAVED_LED, LED_OFF );		// turn the LED off

	else											// Otherwise,
	{
		digitalWrite ( SAVED_LED, LED_ON );			//turn the LED on
		
		if ( DEBUG )
			Serial.println ( "LED On in CheckTimeout" );
	}

	return true;
}


/*
 *	'CheckPulse' is called from the main loop anytime one of the rotator motors
 *	is running and also from the 'Calibrate' function.
 *
 *	This was completely re-written in Version 1.2. In previous versions, the pulses
 *	generated interrupts and 'CheckPulse' did follow up processing when a pulse
 *	was detected. In version 1.2, the interrupt approach was eliminated and pulse
 *	handling is now done completely in here.
 *
 *	The 'INDEX' pins are normally 'HIGH' and go 'LOW' when a pulse is in progress.
 */

void CheckPulse ()
{
	if ( !activeMotor  )						// If neither motor running
		return;									// nothing to do here!

	if (  activeMotor == MTR_AZ )				// Running the azimuth motor?
	{
		if ( digitalRead ( AZ_INDEX ))			// If 'HIGH' no pulse in progress
			return;								// Nope

		delay ( DEBOUNCE );						// Looks like a pulse so wait for debounce time
		if ( digitalRead ( AZ_INDEX ))			// Did it go back to 'HIGH' already?
			return;								// Yes, then assume it's a false pulse


/*
 *	If we get here, we saw the 'INDEX' pin go 'LOW' and it was still 'LOW' after the
 *	'DEBOUNCE' time expired so we assume it's a valid pulse. We wait for the 'INDEX'
 *	pin to go 'HIGH' again, but if it doesn't before the 'MOTOR_TIMEOUT' time
 *	elapses we exit the function. The 'CheckTimeout' function will actually handle
 *	the timeout condition.
 *
 *	Even though the motor is still running, we set the stop time to the current time.
 *	This fixed a bug where the azimuth was being saved while the motor was still
 *	running.
 */

		while ( !digitalRead ( AZ_INDEX ))					// Wait for it to go high
			if (( millis () - pulseTime ) > MOTOR_TIMEOUT )	// Out of time?
				return false;								//Yes!


/*
 *	It's a legit pulse, so adjust the current azimuth appropriately.
 */

		azStopTime = pulseTime = millis ();		// Assume legitimate pulse

		if ( dirRelay == TURN_CW )				// Turning clockwise?
			curAz++;							// Yes, increment the current azimuth
		else
			curAz--;							// Otherwise, decrement the current azimuth

		if ( curAz == tgtAz )					// Target reached?
			StopMotors ( AZ_NORMAL );			// Yes, stop the motor

		lastGoodAz = curAz;						// Remember last good azimuth
		curAzChanged = true;					// Flag for display update
		saveAz = true;							// Also needs to be saved eventually
	}

	if ( ELEVATION && ( activeMotor == MTR_EL ))	// Running the elevation motor?
	{
		if ( digitalRead ( EL_INDEX ))			// Pulse in progress?
			return;								// Nope

		delay ( DEBOUNCE );						// Looks like a pulse so wait for debounce time
		if ( digitalRead ( EL_INDEX ))			// Gone already?
			return;								// False pulse

		while ( !digitalRead ( EL_INDEX ))					// Wait for it to go high
			if (( millis () - pulseTime ) > MOTOR_TIMEOUT )	// Out of time?
				return false;								//Yes!


/*
 *	It's a legit pulse
 */
		elStopTime = pulseTime = millis ();		// Remember when

		if ( dirRelay == MOVE_UP )				// Raising the antennas?
			curEl++;							// Yes, increment the current elevation
		else
			curEl--;							// Otherwise, decrement the current elevation

		if ( curEl == tgtEl )					// Target reached?
			StopMotors ( EL_NORMAL );			// Yes, stop the motor

		lastGoodEl = curEl;						// Remember last good azimuth
		curElChanged = true;					// Flag for display update
		saveEl = true;							// Also needs to be saved eventually
	}

	UpdateNumbers ();							// Give operator status
}


/*
 *	If either of the motors is running, 'CheckReversal' looks to see if the active
 *	motor is still turning in the correct direction. It's possible that a command
 *	from the PC or operation of the manual pushbuttons set a new target azimuth
 *	or elevation that would require the motor to start moving in the other direction.
 *
 *	If we see this is the case, we just stop the rotation, and the fact that the
 *	target and current azimuth and/or elevation are unequal will be picked up in
 *	the next pass through the 'loop' and the motor will be started again in the
 *	correct direction.
 */

void CheckReversal ()
{
	bool reverse = false;						// 'true' if we need to reverse

	if ( !activeMotor )							// Actually rotating?
		return;									// No, return


/*
 *	If the azimuth motor is running in the clockwise direction and the current
 *	azimuth is greater than the target azimuth, we need to reverse. If it's turning
 *	counterclockwise and the current azimuth is less than the target azimuth
 *	we also need a reversal.
 */

	if ( activeMotor == MTR_AZ )
	{
		if (( dirRelay == TURN_CW ) && ( curAz > tgtAz ))
			reverse = true;

		if (( dirRelay == TURN_CCW ) && ( curAz < tgtAz ))
			reverse = true;
	}


/*
 *	Similarly, if the elevation motor is moving up and the target elevation is
 *	less than the current elevation we need to reverse it. If the elevation motor
 *	is moving down and the target elevation is greater than the current elevation
 *	we also need a reversal.
 */

	if ( ELEVATION && ( activeMotor == MTR_EL ))
	{
		if (( dirRelay == MOVE_UP ) && ( tgtEl < curEl ))
			reverse = true;

		if (( dirRelay == MOVE_DOWN ) && ( tgtEl > curEl ))
			reverse = true;
	}

	if ( reverse )
		StopMotors ( REVERSAL );				// Turn motors off
}


/*
 *	The 'Calibrate' function can no longer be initiated by a USB command (Version 1.2).
 *
 *	The procedure is completely changed in Version 1.2 and is thoroughly explained in the
 *	documentation.
 */

void Calibrate ()
{
	if ( !powerStatus )							// If the power supply is off
		return;									// Nothing to do here!


/*
 *	If the rotator is an EL/AZ type, we do the elevation calibration first. The
 *	elevation is calibrated by forcing the motor to run to the zero degree
 *	endstop. If an azimuth only rotator, the elevation code doesn't even compile.
 */

#if ( ELEVATION )

	elIsCalibrating = true;						// Not sure actually needed anymore

	digitalWrite ( SAVED_LED, LED_OFF );		// Turn the saved LED off

	lcdBuffer_1 = elCalibrateString;			// Display elevation calibrating message
	CheckLCD ();

	tgtEl = -180;								// Way beyond the minimum
	tgtElChanged = true;						// Show need to update the display

	UpdateNumbers ();							// Show updated target

	StartMotor ( MTR_EL, MOVE_DOWN );			// Start the motor

	while ( true )								// Until we reach the target
	{

/*
 *	Prior to Version 1.2, a one second timer interrupt was used to invoke the
 *	'UpdateNumbers' function. In Version 1.2, that was eliminated and now we
 *	check to see if anything needs to be updated on the screen.
 */

		if ( tgtAzChanged | curAzChanged | tgtElChanged | curElChanged )
			UpdateNumbers ();

		CheckPulse ();							// If we got an index pulse, process it

		if ( CheckTimeout ())					// Did a timeout occur?
			break;								// Yes - stop here
	}

	elIsCalibrating = false;					// Not sure actually needed anymore

	StopMotors ( EL_CAL );						// Stop the motor

	UpdateNumbers ();							// Final display of target and current azimuth

	lcdBuffer_1 = elCalibratedString;			// Display elevation calibrated message
	CheckLCD ();
	notHelloTime = millis ();

	EEPROM.put ( EL_ADDRESS, curEl );			// Azimuth is saved
	saveEl = false;								// Clear the flag

#endif


/*
 *	Now we do the azimuth. As there are no endstops on the azimuth motor, we assume the
 *	operator has visually determined the antenna's current azimuth and we read the 'CW'
 *	and 'CCW' buttons to adjust the current and target azimuths until the 'CAL' button
 *	is pressed and released again.
 *
 *	The button acceleration is not used here as the assumption is that the azimuths
 *	only need to be corrected by a few degrees at most.
 */

	azIsCalibrating = true;							// Not sure actually needed anymore

	digitalWrite ( SAVED_LED, LED_OFF );			// Turn the saved LED off

	lcdBuffer_1 = azCalibrateString;				// Display azimuth calibrating message
	CheckLCD ();									// Show need to update the display

	UpdateNumbers ();								// Show updated target

	while ( true )									// Until 'CAL' button is pressed and released
	{
		if ( !digitalRead ( CW_BTN ))				// 'CW' button pushed?
		{
			tgtAz++;								// Increment the target

			if ( tgtAz > MAX_AZIMUTH )
				tgtAz = MAX_AZIMUTH;

			curAz = tgtAz;							// and the current azimuth
			tgtAzChanged = curAzChanged = true;		// Both need to be displayed

			UpdateNumbers ();						// Show updated numbers

			delay ( BTN_READ_TIME );				// Wait a bit
		}

		if ( !digitalRead ( CCW_BTN ))				// 'CCW' button pushed?
		{
			tgtAz--;								// Decrement the target

			if ( tgtAz < MIN_AZIMUTH )
				tgtAz = MIN_AZIMUTH;

			curAz = tgtAz;							// and the current azimuth
			tgtAzChanged = curAzChanged = true;		// Both need to be displayed

			UpdateNumbers ();						// Show updated numbers

			delay ( BTN_READ_TIME );				// Wait a bit
		}

		if ( !digitalRead ( CAL_BTN ))				// 'CAL' button pushed?
		{
			while ( !digitalRead ( CAL_BTN )) {}	// Wait for it to be released
				break;								// All done!
		}
	}

	azIsCalibrating = false;						// Not sure actually needed anymore

	lcdBuffer_1 = azCalibratedString;				// Display azimuth calibrated message
	CheckLCD ();
	notHelloTime = millis ();

	EEPROM.put ( AZ_ADDRESS, curAz );				// Azimuth is saved
	saveAz = false;									// Clear the flag
}													// End of 'Calibrate' function


/*
 *	'CheckButtons' looks to see if any of the manual command buttons are pushed.
 *
 *	When a button is pushed, we lock onto it until it is released. We look at it
 *	every 'BTN_READ_TIME' milliseconds to see if it is still pushed.
 *
 *	All we do is check each button every 'BTN_READ_TIME' milliseconds and increment or
 *	decrement the target azimuth or elevation appropriately each time we read it.
 *
 *	The azimuth buttons take priority over the elevation and calibrate buttons.
 *
 *	Changed in Version 1.2. If the 'CAL' button is pressed, we wait for it to be
 *	released before initiating the calibration sequence which was also modified in
 *	Version 1.2.
 *
 *	We can determine if a button is active and which one it is based on the definitions
 *	of the GPIO pins used to read them. The button pins are all analog pins (A8 through
 *	A12). It turns out that there are digital pin definitions that correspond to the 'A'
 *	designations; for the Mega 2560 processor, they are:
 *
 *		CW_BTN		A8  or D62
 *		CAL_BTN		A9  or D63
 *		DOWN_BTN	A10 or D64
 *		UP_BTN		A11 or D65
 *		CCW_BTN		A12 or D66
 *
 *	'NO_BTN' is defined as 0, and the 'activeButton' variable will have that or one
 *	of the above values.
 *
 *	Modified in Version 1.1:
 *
 *		If 'ELEVATION' is 'true', we check all the buttons. If it is false we ignore the
 *		'UP' and 'DOWN' ones.
 */

void CheckButtons ()
{
uint8_t	buttonIncrement = 1;					// Default degrees to change

	if ( !powerStatus )							// If no power
		return;									// Don't change the numbers

	buttonReadTime = millis ();					// Update time last checked

	if ( activeButton != NO_BTN )				// Previously active button?
	{
		if ( !digitalRead ( activeButton ))		// If yes, is it still active?
		{


/*
 *	Here we see if the button accelerator capability is enabled and if it is, we see if
 *	the button has been held sufficently long to engage the acellerator.
 */
 
 			if ( BTN_FAST &&  ( buttonActiveTime != 0 ))
				if (( millis() - buttonActiveTime ) > BTN_FAST_TIME )
					buttonIncrement = BTN_FAST_INCR;

			switch ( activeButton )				// Yes update appropriate target
			{
				case CW_BTN:
					IncAz ( buttonIncrement );	// Increment the target azimuth
					break;

				case CCW_BTN:
					DecAz ( buttonIncrement );	// Decrement the target azimuth
					break;

#if ( ELEVATION )								// Only if elevation rotator

				case UP_BTN:
					IncEl ( buttonIncrement );	// Increment the target azimuth
					break;

				case DOWN_BTN:
					DecEl ( buttonIncrement );	// Decrement the target azimuth
					break;
#endif

			}										// End of switch ( activeButton)
		}											// End of if ( !digitalRead ( activeButton )

		else
		{
			buttonActiveTime = 0;
			activeButton = NO_BTN;					// No active button now
		}
	}												// End of if ( activeButton != NO_BTN )

	if ( activeButton != NO_BTN )					// If there is an active button
	{
		UpdateNumbers ();							// Update the display
		return;										// And we're outta here
	}


/*
 *	If we get here, there was no previously active button, so we need to check them all.
 *	If we find an active one, we increment or decrement the target azimuth or elevation.
 *	Once we find one, we stop looking; in other words you can't press two buttons at the
 *	same time.
 *
 *	Again, if 'ELEVATION' is 'true' check all the buttons. If it is false, don't
 *	check the 'UP' and 'DOWN' ones.
 */

	if ( !digitalRead ( CW_BTN ))				// Request for CW azimuth rotation?
	{
		activeButton = CW_BTN;
		buttonActiveTime = millis ();
		IncAz ( 1 );
		return;
	}

	if ( !digitalRead ( CCW_BTN ))				// Request for CCW azimuth rotation?
	{
		activeButton = CCW_BTN;
		buttonActiveTime = millis ();
		DecAz ( 1 );
		return;
	}

#if ( ELEVATION )								// If no elevation no need to check these

	if ( !digitalRead ( UP_BTN ))				// Request for elevation increase?
	{
		activeButton = UP_BTN;
		buttonActiveTime = millis ();
		IncEl ( 1 );
		return;
	}

	if ( !digitalRead ( DOWN_BTN ))				// Request elevation decrease?
	{
		activeButton = DOWN_BTN;
		buttonActiveTime = millis ();
		DecEl ( 1 );
		return;
	}

#endif

	if ( !digitalRead ( CAL_BTN ))				// Request for calibration?
	{
		if ( !azIsCalibrating && !elIsCalibrating )
		{
			while ( !digitalRead ( CAL_BTN )) {}	// Wait for release
			Calibrate ();
		}
	}

	activeButton = NO_BTN;						// No active buttons
	return;
}												// End of 'CheckButtons'


/*
 *	'GetCommand' reads commands from the USB port.
 *
 *	Some of the commands we recognize are part of the Yaesu GS-232 command language;
 *	some or not. The non-standard commands conform to the Yaesu command format. The
 *	non-standard ones include some for debugging and calibration purposes.
 *
 *	Here's what we recognize and what they do; more detailed explanations can be found
 *	in the documentation.
 *
 *	The order is based on which ones are most often used.
 *
 *		'C'		The 'C' command is used by the N1MM+ Rotor program to request the
 *		'C2'	current azimuth. The 'C2' command is used by SatPC32 to request
 *				both the azimuth and elevation.
 *
 *		'M'		The 'M' command is used by PC programs to specify a new azimuth. The
 *		'W'		'W' command specifies both an azimuth and an elevation.
 *
 *		'E'		The 'E' command is only used in testing. it works like the 'M' command,
 *				except it specifies an elevation. Check the GS-232 manual; might need
 *				to be 'B'.
 *
 *		'F'		The calibration can no longer be started via a command (Version 1.2).
 *
 *		'P'		The 'P' command is used to 'Park' the azimuth and elevation motors at
 *				specific positions. Those positions are defined in the 'SPID.h'
 *				header file.
 *
 *		'Z'		The 'Z' (Zap) command is one I added for testing and debugging. It sets
 *				the current and target azimuth and elevation to the values specified. The
 *				command format is the same as for the 'W' command.
 *
 *	If one of the pushbuttons is active, we disable any commands which would change the 
 *	target azimuth or elevation; in other words, the buttons have priority over commands
 *	from the PC.
 */

bool GetCommand ()
{
	int16_t		tempAz;					// Temporary azimuth in degrees
	int16_t		tempEl;					// Temporary elevation in degrees

	uint8_t		dir;					// 'CW' or 'CCW' (or an alias)

	char 		command;				// The command character (Null to start with)
	String		commandBuffer;			// For reading commands from USB port

	command = '\0';						// The command character (Null to start with)
	commandBuffer = "";					// Buffer is empty

	if ( Serial.available () )			// Anything to read?
	{
		command = toupper ( Serial.peek () );				// Get the command letter
		commandBuffer = Serial.readStringUntil ( '\r' );	// Read the entire command
	}

	if (( command == '\0' ) || ( commandBuffer == "" ))		// If no new command
		return false;										// Nothing to do

	if ( DEBUG )
	{
		Serial.print ( "commandBuffer = " );
		Serial.println ( commandBuffer );
	}

//	LCDPrint ( 1, commandBuffer );				// Temporarily show the command on the display


/*
 *	The "command" variable now contains the single character command letter (in
 *	upper case) and the 'commandBuffer' contains the entire command line (as entered).
 */

	switch ( command )
	{


/*
 *	For the 'C' command, we only need to report the current azimuth in the format
 *	'AZnnn'. The 'C2' command requires us to report both the current azimuth and
 *	elevation in the format '+0nnn+0nnn'.
 */

		case	'C':								// Respond to request for current azimuth
		{											// and maybe elevation
			if ( commandBuffer[1] != '2' )			// If not '2'. it's the plain vanilla 'C' request
			{
				workBuffer = "AZ=";
				workBuffer += Int_2_String ( curAz );
			}

			else									// This is a 'C2' type request
			{
				workBuffer = "+0";
				workBuffer += Int_2_String ( curAz );
				workBuffer += "+0";
				workBuffer += Int_2_String ( curEl );
			}

			workBuffer += '\r';						// Add a carriage return
			Serial.print ( workBuffer );			// Send the answer to the PC


/*
 *	If in debugging mode, we also output a newline which makes the output easier
 *	to read on the screen:
 */

			if ( DEBUG )
				Serial.print ( '\n' );

			Serial.flush ();						// Make sure everything gets sent
			return true;
		}


/*
 *	The 'M' command specifies a new azimuth; the 'W' command specifies both a new
 *	azimuth and elevation. Fortunately, the azimuth value is the same for both commands
 *	so we can handle that part with common code. If the command is a 'W' command we
 *	are going to check the elevation against the "MAX"ELEVATION".
 */

		case	'M':								// Turn to specified azimuth
		case	'W':								// Azimuth + elevation
		{
			if ( !powerStatus )						// If the power supply is off
				return false;						// Can't move the rotator

			if ( activeButton != NO_BTN )			// Is there an active button?
				return false;						// Yes, don't change targets

			workBuffer = commandBuffer.substring ( 1, 4 );	// Should be 3 digit azimuth
			tgtAz = workBuffer.toInt ();					// Make it a number

			if ( tgtAz < MIN_AZIMUTH )				// Shouldn't happen
				tgtAz  = MIN_AZIMUTH;

			if ( tgtAz > MAX_AZIMUTH )				// Also shouldn't happen
				tgtAz  = MAX_AZIMUTH;

			tgtAzChanged = true;					// Signal to update the display

			if ( command == 'W' )					// Elevation also specified?
			{
				if ( ELEVATION )
				{
					workBuffer = commandBuffer.substring ( 5, 8 );	// Should be 3 digit elevation
					tgtEl = workBuffer.toInt ();					// Make it a number

					if ( tgtEl > MAX_ELEVATION )		// Too big?
						tgtEl  = MAX_ELEVATION;			// Yep, adjust it

					if ( tgtEl < MIN_ELEVATION )		// Too small?
						tgtEl  = MIN_ELEVATION;			// Yep, adjust it

					tgtElChanged = true;				// Needs to be displayed
				}
			}

			if ( DEBUG )
			{
				Serial.print ( "GetCommand - tgtAz = " );
				Serial.print ( tgtAz );

				if ( ELEVATION )						// Only if elevation capable rotator
				{
					Serial.print ( ",  tgtEl = " );
					Serial.print ( tgtEl );
				}

				Serial.print ( '\n' );					// Add a newline
			}

			return true;
		}


/*
 *	The 'E' command specifies a new elevation (for testing and degugging only)
 */

#if ( ELEVATION )									// Command disabled if no elevation

		case	'E':								// Move to specified elevation
		{
			if ( !powerStatus )						// If the power supply is off
				return false;						// Can't move the rotator

			if ( activeButton != NO_BTN )			// Is there an active button?
				return false;						// Yes, don't change targets

			workBuffer = commandBuffer.substring ( 1, 4 );	// Should be 3 digit elevation
			tgtEl = workBuffer.toInt ();					// Make it a number

			if ( tgtEl < MIN_ELEVATION)				// Shouldn't happen
				tgtEl  = MIN_ELEVATION;

			if ( tgtEl > MAX_ELEVATION )			// Also shouldn't happen
				tgtEl  = MAX_ELEVATION;

			tgtElChanged = true;					// Signal to update the display

			return true;
		}

#endif


/*
 *	The 'P' command parks the antenna at the azimuth and elevation specified by the
 *	'AZ_PARK' and 'EL_PARK' symbols defined in the 'SPID.h' header file. All we
 *	do is set the targets as specified. See the documentation for a complete explanation.
 */

		case 'P':
		{
			if ( !powerStatus )						// If the power supply is off
				return false;						// Can't move the rotator

			if ( activeButton != NO_BTN )			// Is there an active button?
				return false;						// Yes, don't change targets

			#ifdef AZ_PARK
				tgtAz = AZ_PARK;
			#endif

			#ifdef EL_PARK
				if ( ELEVATION )
					tgtEl = EL_PARK;
			#endif

			tgtElChanged = true;					// Signal to update the display
			tgtAzChanged = true;

			return true;
		}


/*
 *	The 'Z' (Zap) command is used for testing and debugging. The command format is the
 *	same as that of the 'W' command. It causes the current and target azimuths and
 *	elevations to be set to the specified values. The values are compared to the limits.
 *
 *	It can be used even when the motor power supply is turned off.
 */

		case	'Z':									// Force azimuth + elevation
		{
			if ( activeButton != NO_BTN )				// Is there an active button?
				return false;							// Yes, don't change targets

			workBuffer = commandBuffer.substring ( 1, 4 );	// Should be 3 digit azimuth
			tgtAz = workBuffer.toInt ();					// Make it a number

			if ( tgtAz < MIN_AZIMUTH )					// Shouldn't happen
				tgtAz  = MIN_AZIMUTH;

			if ( tgtAz > MAX_AZIMUTH )					// Also shouldn't happen
				tgtAz  = MAX_AZIMUTH;

			curAz = tgtAz;								// Current same as target

			tgtAzChanged = true;						// Signal to update the display
			curAzChanged = true;						// Signal to update the display


/*
 *	Here is the one case where we allow the target and current elevations to be
 *	altered even when the rotator is not elevation capable.
 */
			workBuffer = commandBuffer.substring ( 5, 8 );	// Should be 3 digit elevation
			tgtEl = workBuffer.toInt ();					// Make it a number

			if ( tgtEl > MAX_ELEVATION )			// Too big?
				tgtEl  = MAX_ELEVATION;				// Yep, adjust it

			if ( tgtEl < MIN_ELEVATION )			// Too small?
				tgtEl  = MIN_ELEVATION;				// Yep, adjust it

			curEl = tgtEl;							// Current same as target

			tgtElChanged = true;					// Needs to be displayed
			curElChanged = true;					// Needs to be displayed

			EEPROM.put ( AZ_ADDRESS, curAz );		// Save both azimuth
			EEPROM.put ( EL_ADDRESS, curEl );		// and elevation
			saveAz = false;							// Clear the flags
			saveEl = false;

			return true;
		}


		default:
		{
			return false;
			break;
		}
	}											// End of command processing 'switch'
}												// End of 'GetCommand'


/*
 *	'SayHello' checks to see if the default  message is already on line 1 of the display, and if
 *	so, just returns. It also returns if the 'helloTimeout' since the time some other type of
 *	message was displayed has not expired.
 */

void SayHello ()
{
	String	Temp_Buffer;

	if ( helloMsg || activeMotor )					// Already displaying message or rotating?
		return;										// Don't need to do it again

	helloTime = millis ();							// Current run time

	if ( helloTime - notHelloTime < helloTimeout )	// Time expired?
    	return;										// Nope!

	LCDPrint ( 1, helloString );					// We're on the air!
	LCDPrint ( 2, headerString );					// Display header fields
	LCDPrint ( 3, azHeader );
	if ( ELEVATION )
		LCDPrint ( 4, elHeader);					// Only if elevation type rotator

	tgtAzChanged   = true;							// Force update of all the numbers
	tgtElChanged   = true;
	curAzChanged   = true;
	curElChanged   = true;

	UpdateNumbers ();

	helloMsg = true;								// Message is displayed
}


/*
 *	'UpdateNumbers' checks the four flags that indicate whether the current or target
 *	azimuth or elevation have changed and updates the numbers on the screen accordingly.
 */

void UpdateNumbers ()
{
	int16_t		tempDegrees;						// Temporary degrees
	String		degString;							// Contains just the number in ASCII
	char		lcdString[4];						// Need a regular array for print function

	if ( tgtAzChanged )								// If the target azimuth needs updated
	{
		degString = Int_2_String ( tgtAz );			// String for the display
		degString.toCharArray ( lcdString, 4 );		// Copy data to internal buffer

		lcdDisplay.setCursor ( 16, 2 );				// 17th character position on line 3
		lcdDisplay.printstr ( lcdString );			// Display it

		tgtAzChanged = false;						// Clear the flag
	}

	if ( curAzChanged )								// If the current azimuth needs updated
	{
		degString = Int_2_String ( curAz );			// String for the display
		degString.toCharArray ( lcdString, 4 );		// Copy data to internal buffer

		lcdDisplay.setCursor ( 11, 2 );				// 12th character position on line 3
		lcdDisplay.printstr ( lcdString );			// Display it

		curAzChanged = false;						// Clear the flag
	}


/*
 *	When calibrating the elevation motor, the target elevation is forced to -180 degrees. 
 *	It will also be possible for the current elevation to go negative, thus we need to
 *	adjust the negative numbers to positive ones in the range of 0 to 360 degrees.
 */

	if ( tgtElChanged && ELEVATION )					// If the target elevation needs updated
	{
		if ( tgtEl < 0 )								// Negative number?
			degString = Int_2_String ( tgtEl + 360 );	// Yes, add 360 degrees
		else
			degString = Int_2_String ( tgtEl );			// Otherwise leave number unchanged

		degString.toCharArray ( lcdString, 4 );			// Copy data to internal buffer

		lcdDisplay.setCursor ( 16, 3 );					// 17th character position on line 4
		lcdDisplay.printstr ( lcdString );				// Display it

		tgtElChanged = false;							// Clear the flag
	}

	if ( curElChanged && ELEVATION )					// If the current elevation needs updated
	{
		if ( curEl < 0 )								// Negative number?
			degString = Int_2_String ( curEl + 360 );	// Yes, add 360 degrees
		else
			degString = Int_2_String ( curEl );			// Otherwise leave number unchanged

		degString.toCharArray ( lcdString, 4 );			// Copy data to internal buffer

		lcdDisplay.setCursor ( 11, 3 );					// 12th character position on line 4
		lcdDisplay.printstr ( lcdString );				// Display it

		curElChanged = false;							// Clear the flag
	}
}														// End of 'UpdateNumbers'


/*
 *	'CheckLCD' looks for data in the LCD buffers, and if anything is in either of,
 *	the ones for line 1 or line 2 it uses 'LCDPrint' to display them. Lines 3 and 4
 *	are handled separately.
 */

void CheckLCD ()
{
	if ( lcdBuffer_1.length ())				// Anything in line 1 buffer?
	{
		LCDPrint ( 1, lcdBuffer_1 );		// Yep!, display it
		lcdBuffer_1 = "";					// Clear the buffer
		helloMsg = false;					// We dont use this to display hello msg
		notHelloTime = millis ();			// Time stamp
	}

	if ( lcdBuffer_2.length ())				// Anything in line 2 buffer?
	{
		LCDPrint ( 2, lcdBuffer_2 );		// Yep!, display it
		lcdBuffer_2 = "";					// Clear the buffer
	}
}											// End of 'CheckLCD'


/*
 *	'LCDPrint' displays something on the LCD display. Arguments are which line to put it
 *	on, and what to put there. Note, we allow the caller to specify lines 1 through 4, and
 *	convert those to 0 through 3.
 */

void LCDPrint ( int line, String info )
{
	int		i;									// Loop counter
	int	 	lineNumber;							// Adjusted line number
	int		bufferLength;						// Actual number of characters in the string
	char	buffer[LCD_WIDTH + 4];				// Need a regular array for print function

	if ( line < 1 || line > LCD_HEIGHT )		// Range check
		return;

	lineNumber = line - 1;						// Adjust requested line number
	bufferLength = info.length ();				// Length of data

	if ( bufferLength > LCD_WIDTH )				// Make sure not too long
		bufferLength = LCD_WIDTH;				// If too long, truncate

	info.toCharArray ( buffer, bufferLength + 1 );		// Copy data to internal buffer

	if ( bufferLength < LCD_WIDTH )						// Need padding?
		for ( i = bufferLength; i < LCD_WIDTH; i++ )	// Yep!
			buffer[i] = ' ';

	buffer[LCD_WIDTH] = '\0';					// Need a null at the end

	lcdDisplay.setCursor ( 0, lineNumber );		// 1st character position on lineNumber
	lcdDisplay.printstr ( buffer );				// Display it
}												// End of 'LCDPrint'


/*
 *	'Int_2_String' converts an integer (azimuth or elevation) into a 3 digit
 *	'String' with the appropriate number of leading zeroes.
 */

String	Int_2_String ( int16_t number )
{
	String	answer;							// Build the answer here

	if ( number < 10 )						// Single digit number?
		answer = "00";						// Then 2 leading zeroes

	else if ( number < 100 )				// 2 digit number?
		answer = "0";						// Only 1 leading zero

	answer += number;						// Add on the actual number

	return answer;							// And return it
}


/*
 *Whenever a timeout occurs, we flash the 'SAVED_LED'.
 */

void BlinkLED ()
{
	for ( int i = 0; i < 5; i++ )
	{
		digitalWrite ( SAVED_LED, LED_OFF );		// Turn the azimuth saved LED off
		delay ( 100 );								// Wait 100 milliseconds
		digitalWrite ( SAVED_LED, LED_ON );			// Turn the azimuth saved LED on
		delay ( 100 );								// Wait another 100 milliseconds
	}
}


/*
 *	These functions increment or decrement the target azimuth and elevation:
 */

void IncAz ( uint8_t amount )
{
	tgtAz += amount;					// Increment the target azimuth
	if ( tgtAz > MAX_AZIMUTH )			// Range check
 		tgtAz = MAX_AZIMUTH;			// Set upper limit
	tgtAzChanged = true;				// Display needs updated
}

void DecAz ( uint8_t amount )
{
	tgtAz -= amount;					// Decrement the target azimuth
	if ( tgtAz < MIN_AZIMUTH )			// Range check
 		tgtAz = MIN_AZIMUTH;			// Set lower limit
	tgtAzChanged = true;				// Display needs updated
}

void IncEl ( uint8_t amount )
{
	tgtEl += amount;					// Increment the target elevation
	if ( tgtEl > MAX_ELEVATION )		// Range check
 		tgtEl = MAX_ELEVATION;			// Set upper limit
	tgtElChanged = true;				// Display needs updated
}

void DecEl ( uint8_t amount )
{
	tgtEl -= amount;					// Decrement the target elevation
	if ( tgtEl < MIN_ELEVATION )		// Range check
 		tgtEl = MIN_ELEVATION;			// Set upper limit
	tgtElChanged = true;				// Display needs updated
}


/*
 *	"CheckReboot" looks to see if the operator pressed the reset button on the back of the
 *	controller, and if so, saves the current azimuth and does a software initiated restart.
 */

void CheckReboot ()
{
	if ( !digitalRead ( REBOOT ))				// Operator want to reboot?
	{
		StopMotors ( SW_REBOOT );				// Make sure motors stopped

		EEPROM.put ( AZ_ADDRESS, curAz );		// Save azimuth
		EEPROM.put ( EL_ADDRESS, curEl );		// and elevation

		BlinkLED ();							// Flash the  saved LED

		soft_restart ();						// Reboot
	}
}
