/*
 Name:		CyEnduranceTester.ino
 Created:	8/6/2019 9:32:28 AM
 Author:	jevans
 Notes:		This code is used to control arduino for cy endurance tester.  Project folder is 
			30522.  Reference this folder for 3d models and additional details.  
			Used Arduino MEGA, AnyMark AM-3637 motor, motor shield powered by 12v supply, (2) AFC A211B-12M solenoids,
			2 limit switches, and 2 push buttons for start and stop.
*/


#include <LiquidCrystal.h>
#include <Adafruit_MotorShield.h>
#include <MD_UISwitch.h>
#include <EEPROM.h>


const int analogSpeedPin = A11;  //connected to pot for speed control
const int analogSol1Pin = A9;
const int analogSol2Pin = A10;
const int RTDSensorPin = A12;
const int analogLCDSwitchPin = A0; // switch pin for lcd display

// default dwell times; these can be updated using lcd display
int upperLimDwell = 2000;	// dwell time at upper limit switch
int lowerLimDwell = 2000;	// dwell time at lower limit switch
int sol1Dwell = 2000;		// dwell time at solenoid 1 (pressurize)
int sol2Dwell = 2000;		// dwell time at solenoid 2 (vent)
int maxSpeed = 75;
int toggleSpeed = 75;
int cycleCount = 0;
int cycleCountPrev = -1;
int testPaused = 0;
int maxCycles = 200;		// maximum number of cycles to execute
bool upperLimitReached = false;
bool lowerLimitReached = false;
bool manMtrUpOn = false;
bool manMtrDownOn = false;
long iterations = 1000;
long startTime = 0;
long elapsedTimePausedTest = 0;
long prevTimeSaveConfig = millis();
bool testActive = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// states
enum state_enum {
	INACTIVE,
	RAISE_MOTOR,
	LOWER_MOTOR,
	PRESSURIZE,
	VENT
};

int state = INACTIVE;

// setting definition
typedef struct {
	char *title;
	int value;
	int min;
	int max;
	int step;
	bool isStd;
}setting;

enum setting_enum
{
	MOTOR_SPEED,
	UPPER_DWELL,
	LOWER_DWELL,
	SOL1_DWELL,
	SOL2_DWELL,
	TGL_SPEED,
	RESET_COUNTER,
	MAX_CYCLES,
	HOME
};

// settings structure - defined in initializeSettings();
setting settings[9] = {};

// save settings in EEPRM
#define CONFIG_START 32

typedef struct {
	int testInt;
	int cfgCycleCount;
	int cfgMotorSpeed;
	int cfgUpperDwell;
	int cfgLowerDwell;
	int cfgSol1Dwell;
	int cfgSol2Dwell;
	int cfgTglSpeed;
	int cfgMaxCycles;
} configuration_type;

// default values
configuration_type CONFIGURATION = {};

// function declarations
int loadConfig();
void initializeSettings();
void saveConfig();
int read_LCD_buttons();
void AccelerateMotor(int direction, int speed = maxSpeed);
void SetDefaultLCDFormat();
void PrintSaveMsg();
void PrintVentingMsg();
long GetElapsedMillis();
long resetStartTime(long prevElapsedTime);
void delaySaveConfig();
void LCD_Button_Action();
void GetElapsedTimeStr(char* outStr);
float GetTemperatureF();
void CheckButtons();
int PressurizeSolenoid(int pin);
int VentSolenoid(int pin);
void StopMotor(int maxSpeed);
int RaiseMotorToUpperLim();
int LowerMotorToLowerLim();
int GetMotorSpeed();
void PauseTest();
void SafelyPauseExecution();
void SafelyResumeExecution();
void InteractiveLCD();

// buttons connected to digital io pins
const int NUM_BUTTONS = 6;
enum button_enum {
	GREEN_BUTTON,
	RED_BUTTON,
	UPPER_LIMIT,
	LOWER_LIMIT,
	MANMOTOR_UP,
	MANMOTOR_BACKWARD
};

const int pins[NUM_BUTTONS] = {
  22,           // green button
  24,           // red button
  26,           // upper limit switch
  28,            // lower limit switch
  29,			// manual motor forward (toggle switch)
  30			// manual motor backwards (toggle switch)

};
bool pushed[NUM_BUTTONS];



// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// define some values used by the panel and buttons
int lcd_key = 0;
int adc_key_in = 0;
#define btnRIGHT  17 // was 1, updated to reflect char values 'R', 'U', 'D', 'L', 'S', etc
#define btnUP     20
#define btnDOWN   3
#define btnLEFT   11
#define btnSELECT 18
#define btnNONE   0

// --------------------------------------------------------
//  New LCD button read functions using MD_UISwitch library
MD_UISwitch_Analog::uiAnalogKeys_t kt[] =
{
	//{voltage, tolerance, return character}
	  {  20, 20, 'R' },  // Right converts to 17 (btnRIGHT)
	  { 100, 50, 'U' },  // Up converts to 20 (btnUP)
	  { 256, 50, 'D' },  // Down converts to 3 (btnDOWN)
	  { 409, 50, 'L' },  // Left converts to 11 (
	  { 641, 50, 'S' },  // Select converts to 18
};

MD_UISwitch_Analog S(analogLCDSwitchPin, kt, ARRAY_SIZE(kt));


void setup(void)
{
	// LCD Setup
	lcd.begin(16, 2);
	lcd.setCursor(0, 0);
	lcd.print("Starting up...");

	startTime = millis();
	
	Serial.begin(9600);
	// initialize motor
	AFMS.begin();
	myMotor->setSpeed(0);
	myMotor->run(FORWARD);
	myMotor->run(RELEASE);

	// LCD button setup
	S.begin();
	// Set time to detect lcd button press in ms;  Default is 80
	S.setPressTime(30);

	// Load configuration setting from previous use
	if (loadConfig()) {
		Serial.println("Config loaded:");
		Serial.print("CONFIG: cycleCount = "); Serial.println(cycleCount);
		Serial.print("CONFIG: maxSpeed = "); Serial.println(maxSpeed);
		Serial.print("CONFIG: upperLimDwell = "); Serial.println(upperLimDwell);
		Serial.print("CONFIG: lowerLimDwell = "); Serial.println(lowerLimDwell);
		Serial.print("CONFIG: sol1Dwell = "); Serial.println(sol1Dwell);
		Serial.print("CONFIG: sol2Dwell = "); Serial.println(sol2Dwell);
		Serial.print("CONFIG: toggleSpeed = "); Serial.println(toggleSpeed);
		Serial.print("CONFIG: maxCycles = "); Serial.println(maxCycles);
	}
	else {
		Serial.println("Config not loaded!");
		saveConfig();
	}
	// Initialize pins for buttons
	for (int i = 0; i < NUM_BUTTONS; i++)
	{
		pinMode(pins[i], INPUT_PULLUP);
		pushed[i] = false;
	}

	// Show "Starting up" screen for at least two seconds
	while ((millis() - startTime) < 2000);

	// Set the default LCD screen
	SetDefaultLCDFormat();
}



void loop(void)
{
	state = INACTIVE;
	CheckButtons();
	//maxSpeed = GetMotorSpeed();
	LCD_Button_Action();
	long startTime = millis();


	while (testActive && testPaused < 2 && cycleCount < maxCycles) {

		CheckButtons();
		LCD_Button_Action();
		delaySaveConfig();
		// raise motor to upper limit switch
		Serial.println("Raising motor to upper limit switch. 1");
		state = RAISE_MOTOR;
		if (RaiseMotorToUpperLim() > 0) {
			// stop button hit
			return;
		}

		// Dwell at upper lim
		startTime = millis();
		while ((millis() - startTime) < upperLimDwell) {
			CheckButtons();
			LCD_Button_Action();
		}

		//delay(upperLimDwell);

		state = LOWER_MOTOR;
		// lower to lower limit switch
		Serial.println("Lower to lower limit switch 1");
		if (LowerMotorToLowerLim() > 0) {
			// stop button hit
			return;
		}

		// Dwell at lower limit
		startTime = millis();
		while ((millis() - startTime) < lowerLimDwell) {
			CheckButtons();
			LCD_Button_Action();
		}


		//open high pressure solenoid to pressurize valve
		//delay for a period of time to build up pressure
		//Close high pressure solenoid
		state = PRESSURIZE;
		if (PressurizeSolenoid(analogSol1Pin) > 0) {
			return;
		}



		state = RAISE_MOTOR;
		// Raise to upper lim
		Serial.println("Raising motor to upper limit switch. 2");
		if (RaiseMotorToUpperLim() > 0) {
			// stop button hit
			return;
		}

		// Dwell at upper lim
		startTime = millis();
		while ((millis() - startTime) < upperLimDwell) {
			CheckButtons();
			LCD_Button_Action();
		}
		
		//delay(upperLimDwell);

		state = LOWER_MOTOR;
		// Lower to lower limit
		Serial.println("Lower to lower limit switch 2");
		if (LowerMotorToLowerLim() > 0) {
			// stop button hit
			return;
		}

		state = VENT;
		if (VentSolenoid(analogSol2Pin) > 0) {
			return;
		}

		// Dwell at lower limit
		startTime = millis();
		while ((millis() - startTime) < lowerLimDwell) {
			CheckButtons();
			LCD_Button_Action();
		}		
		
		//delay(lowerLimDwell);

		// Increment counter
		cycleCount++;
		Serial.print("Cycle count = ");
		Serial.println(cycleCount);
		if (cycleCount == maxCycles) {
			testActive = false;
		}
		LCD_Button_Action();

	}

}

// load whats in EEPROM in to the local CONFIGURATION if it is a valid setting
int loadConfig() {
	configuration_type newConfig;
	// is it correct?  Should start with 'A'
	EEPROM.get(CONFIG_START, newConfig);
	if (newConfig.testInt == 99) {
		CONFIGURATION = newConfig;
		cycleCount = newConfig.cfgCycleCount;
		maxSpeed = newConfig.cfgMotorSpeed;
		upperLimDwell = newConfig.cfgUpperDwell;
		lowerLimDwell = newConfig.cfgLowerDwell;
		sol1Dwell = newConfig.cfgSol1Dwell;
		sol2Dwell = newConfig.cfgSol2Dwell;
		toggleSpeed = newConfig.cfgTglSpeed;
		maxCycles = newConfig.cfgMaxCycles;
		initializeSettings();
		return 1; // return 1 if config loaded
	}
	initializeSettings();
	return 0; // return 0 if not loaded
}

void initializeSettings() {
	settings[0] = { .title = "> Motor Speed   ",
				.value = maxSpeed,
				.min = 0,
				.max = 255,
				.step = 2,
				.isStd = true };


	settings[1] = { .title = "> Upper Dwell   ",
				.value = upperLimDwell,
				.min = 100,
				.max = 10000,
				.step = 100,
				.isStd = true };

	settings[2] = { .title = "> Lower Dwell   ",
				.value = lowerLimDwell,
				.min = 100,
				.max = 10000,
				.step = 100,
				.isStd = true };

	settings[3] = { .title = "> Sol1 Dwell   ",
				.value = sol1Dwell,
				.min = 100,
				.max = 30000,
				.step = 100,
				.isStd = true };

	settings[4] = { .title = "> Sol2 Dwell   ",
				.value = sol2Dwell,
				.min = 100,
				.max = 30000,
				.step = 100,
				.isStd = true };

	settings[5] = { .title = "> Toggle Speed  ",
				.value = toggleSpeed,
				.min = 5,
				.max = 255,
				.step = 10,
				.isStd = true };

	settings[6] = { .title = "> Reset Counter ",
				.value = 75,
				.min = 5,
				.max = 255,
				.step = 10,
				.isStd = false };

	settings[7] = { .title = "> Max Cycles   ",
				.value = maxCycles,
				.min = 5,
				.max = 4000,
				.step = 10,
				.isStd = true };
	settings[8] = { .title = "> HOME  ",
				.value = 75,
				.min = 5,
				.max = 255,
				.step = 10,
				.isStd = false };
}

// Save configuration data to EEPROM
void saveConfig() {
	CONFIGURATION.cfgCycleCount = cycleCount;
	CONFIGURATION.cfgLowerDwell = lowerLimDwell;
	CONFIGURATION.cfgUpperDwell = upperLimDwell;
	CONFIGURATION.cfgMotorSpeed = maxSpeed;
	CONFIGURATION.cfgSol1Dwell = sol1Dwell;
	CONFIGURATION.cfgSol2Dwell = sol2Dwell;
	CONFIGURATION.cfgTglSpeed = toggleSpeed;
	CONFIGURATION.cfgMaxCycles = maxCycles;
	CONFIGURATION.testInt = 99;
	EEPROM.put(CONFIG_START, CONFIGURATION);
	Serial.println("New configuration file written to EEPROM");
}

int read_LCD_buttons() {
	MD_UISwitch::keyResult_t k = S.read();

	switch (k)
	{
	case MD_UISwitch::KEY_NULL:      /* Serial.print("KEY_NULL"); */  break;
	case MD_UISwitch::KEY_UP:        Serial.print("\nKEY_UP ");     break;
	case MD_UISwitch::KEY_DOWN:      Serial.print("\n\nKEY_DOWN ");   break;
	case MD_UISwitch::KEY_PRESS:     Serial.print("\nKEY_PRESS ");  break;
	case MD_UISwitch::KEY_DPRESS:    Serial.print("\nKEY_DOUBLE "); break;
	case MD_UISwitch::KEY_LONGPRESS: Serial.print("\nKEY_LONG   "); break;
	case MD_UISwitch::KEY_RPTPRESS:  Serial.print("\nKEY_REPEAT "); break;
	default:                         Serial.print("\nKEY_UNKNWN "); break;
	}

	if (k == MD_UISwitch::KEY_PRESS) {
		if (S.getKey() >= ' ') {
			return (char)S.getKey() - 'A';
		}
	}
	return btnNONE;
}

// Accelerate Motor up to speed.  direction=FORWARD=1 -> down; direction=BACKWARD=2 -> up
void AccelerateMotor(int direction, int speed = maxSpeed) {
	myMotor->run(direction);
	for (int i = 0; i <= maxSpeed; i++)
	{
		myMotor->setSpeed(i);
		delay(5);
	}

}

// Update LCD display
// FORMAT: "XXXX MIN; XX.XF "
//		   "CYCLES: XXXXXXXX" 
void SetDefaultLCDFormat() {
	lcd.setCursor(0, 0);	// move cursor to line 0, 0 spaces over
	lcd.print("                ");
	char str[10] = "ERR";
	GetElapsedTimeStr(str);			// updates str with "XXXX MIN" format.  Updates time format between sec, min, hrs.
	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0, 0);
	lcd.print(str);

	lcd.setCursor(11, 0);
	lcd.print(GetTemperatureF());
	lcd.setCursor(15, 0);
	lcd.print("F");
	lcd.setCursor(0, 1);
	lcd.print("Cycle:          ");
	lcd.setCursor(7, 1);
	lcd.print(cycleCount);

}

void PrintSaveMsg() {
	lcd.setCursor(11, 1);
	lcd.print("SAVED ");
}

void PrintVentingMsg() {
	lcd.setCursor(0, 0);
	lcd.print("VENTING          ");
}

// returns the number of milliseconds from startTime to now.  startTime should be defined.
long GetElapsedMillis() {
	return (millis() - startTime);
}

// Calculates new "startTime" to resume timer after pausing test
//		prevElapsedTime: time test has been running in milliseconds
//		returns new startTime in ms
long resetStartTime(long prevElapsedTime) {
	return (millis() - prevElapsedTime);
}

// Saves config data to EEPROM after one hour of elapsed time
void delaySaveConfig() {
	long timeSinceSave = millis() - prevTimeSaveConfig;
	if ((timeSinceSave / 1000 / 3600) > 1) {
		prevTimeSaveConfig = millis();
		saveConfig();
	}
}

void LCD_Button_Action() {
	iterations++;

	if (testActive) {
		lcd.setCursor(0, 0);			// move cursor to line 0, 0 spaces over
		char str[8] = "ERR";
		GetElapsedTimeStr(str);			// updates str with "XXXX MIN" format.  Updates time format between sec, min, hrs.
		lcd.print(str);
	}
	//lcd.print(";");


	if (iterations % 100 == 0) {		// Update temperature every 100 iterations
		/*Serial.print("Iterations = ");
		Serial.println(iterations);*/
		lcd.setCursor(11, 0);
		float temp = GetTemperatureF();
		if (temp > -20) {
			lcd.print(GetTemperatureF());
		}
		else {
			lcd.print(" - ");
		}
		//lcd.print(GetTemperatureF());
		lcd.setCursor(15, 0);
		lcd.print("F");


		if (iterations > 300000) {		// Reset counter if it gets too large
			iterations = 1001;
		}
	}




	if (cycleCount > cycleCountPrev) {
		/*lcd.setCursor(0, 1);
		lcd.print("CYCLE:          ");*/
		lcd.setCursor(7, 1);
		lcd.print(cycleCount);
	}

	lcd.setCursor(0, 0);
	lcd_key = read_LCD_buttons(); // Read the buttons


	switch (lcd_key) {

	case btnRIGHT:
	{
		//lcd.print("RESET COUNTER???");
		//delay(500);
		//lcd_key = read_LCD_buttons();
		//while (lcd_key == btnNONE)
		//{
		//	lcd_key = read_LCD_buttons();
		//}
		////lcd.setCursor(0, 0);
		////lcd.print("lcdkey:         ");
		////lcd.setCursor(9, 0);
		////lcd.print(analogRead(0));
		////delay(2000);
		//if (lcd_key == btnSELECT)
		//{
		//	cycleCount = 0;
		//	cycleCountPrev = -1;
		//	lcd.setCursor(0, 0);
		//	lcd.print("CNTER IS RESET-0");
		//	delay(2000);
		//	SetDefaultLCDFormat();
		//	return;
		//}
		break;
	}
	case btnDOWN:
		InteractiveLCD();
		break;
	}
	cycleCountPrev = cycleCount;
}

void GetElapsedTimeStr(char* outStr) {
	long seconds = (GetElapsedMillis() / 1000);

	if (seconds < 120)
	{
		//sprintf(str, "%4d sec", seconds);

		snprintf(outStr, 9, "%4ld sec", seconds);
	}
	else if (seconds >= 120 && seconds < 3600)
	{
		//sprintf(str, "%4d min", seconds/60);
		int min = seconds / 60;
		int sec = seconds % 60;
		//sprintf(str, "%d min", seconds/60);
		snprintf(outStr, 10, "%3dm%3ds", min, sec);
	}
	else if (seconds >= 3600)
	{
		int totalMin = seconds / 60;
		int hrs = totalMin / 60;
		int min = totalMin % 60;

		//sprintf(str, "%4d hrs", seconds / 3600);
		snprintf(outStr, 9, "%3dh%3dm", hrs, min);
	}
	else {
		snprintf(outStr, 9, "ERR");
	}
}

float GetTemperatureF() {
	//getting the voltage reading from the temperature sensor
	int reading = analogRead(RTDSensorPin);
	// converting that reading to voltage, for 3.3v arduino use 3.3
	float voltage = reading * 5.0;
	voltage /= 1024.0;
	// now print out the temperature



	float temperatureC = (voltage - 0.5) * 100;  //converting from 10 mv per degree wit 500 mV offset	
												 //to degrees ((voltage - 500mV) times 100)
	float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
	float value = (int)(temperatureF * 100 + .5);
	//Serial.print(voltage); Serial.println(" volts.");
	//Serial.print(temperatureC); Serial.println(" deg C.");
	//Serial.print(temperatureF); Serial.println(" deg F.");
	return (float)value / 100;
}

void CheckButtons() {
	// check for button presses
	for (int i = 0; i < NUM_BUTTONS; i++)
	{
		bool previous = pushed[i];
		pushed[i] = digitalRead(pins[i]) == LOW;

		if (pushed[i] != previous)
		{
			if (pushed[i])
			{ // button just pressed
				switch (i)
				{
				case GREEN_BUTTON:
					Serial.println("Green button was pressed.");
					// do action for blue button being set
					if (testActive == false) {
						testActive = true;
						if (testPaused == 1) {
							testPaused = 0;
							startTime = elapsedTimePausedTest;
						}
						else {
							startTime = millis();
						}
					}

					break;

				case UPPER_LIMIT:
					// action for upper limit switch being activated
					Serial.print("[");
					Serial.print(cycleCount);
					Serial.println("] Upper limit switch was pressed.");
					upperLimitReached = true;
					break;

				case LOWER_LIMIT:
					// action for upper limit switch being activated
					Serial.print("[");
					Serial.print(cycleCount);
					Serial.println("] Lower limit switch was pressed.");
					lowerLimitReached = true;
					break;

				case RED_BUTTON:
					// do action for red button being set
					Serial.print("[");
					Serial.print(cycleCount);
					Serial.println("] Red button was pressed. Test terminated");
					testActive = false;
					testPaused++;
					Serial.print("testPaused = ");
					Serial.println(testPaused);
					if (testPaused == 1) {
						elapsedTimePausedTest = GetElapsedMillis();
						PauseTest();
					}
					else {
						saveConfig();
						//startTime = elapsedTimePausedTest;
						VentSolenoid(analogSol2Pin);
					}
					// Updated 8/13
					upperLimitReached = false;
					lowerLimitReached = false;
					break;

				case MANMOTOR_UP:
					//
					Serial.println("ForwardToggle Activated.");
					if (!manMtrUpOn) {
						AccelerateMotor(BACKWARD, toggleSpeed);
						//AccelerateMotorBackward(toggleSpeed);
						manMtrUpOn = true;
					}
					break;

				case MANMOTOR_BACKWARD:
					//
					Serial.println("BackwardToggle Activated.");
					if (!manMtrDownOn) {
						//AccelerateMotorForward(maxSpeed / 2);
						AccelerateMotor(FORWARD, toggleSpeed);
						manMtrDownOn = true;
					}
					break;


				}
			}
			else
			{ // button just release
				switch (i)
				{
				case MANMOTOR_UP:
					// Stop motor when forward toggle released
					Serial.println("Forward toggle released.");
					StopMotor(maxSpeed / 2);
					manMtrUpOn = false;
					break;

				case MANMOTOR_BACKWARD:
					// Stop motor when backward toggle released
					Serial.println("Reverse toggle released.");
					StopMotor(maxSpeed / 2);
					manMtrDownOn = false;
					break;


				case GREEN_BUTTON:
					// do action for green button being released
			  //            Serial.print("[");
			  //            Serial.print(counter);
			  //            Serial.println("] Green button was released.");
					break;

				case RED_BUTTON:
					// do action for red button being released
			  //            Serial.print("[");
			  //            Serial.print(counter);
			  //            Serial.println("] Red button was released.");
					break;

				case UPPER_LIMIT:
					//            Serial.print("[");
					//            Serial.print(counter);
					//            Serial.println("] Upper limit switch was released.");
					break;

				}
			}
			delay(5);
		}
	}
}

int PressurizeSolenoid(int pin) {

	long start = millis();

	Serial.println("Solenoid 1 opened - Pressurizing");
	analogWrite(pin, 255);

	// Check for button presses while holding sol 1 open
	while ((millis() - start) < sol1Dwell) {
		CheckButtons();
		LCD_Button_Action();
	}
	// Close solenoid 1
	analogWrite(pin, 0);
	Serial.println("Solenoid 1 closed");

	// If stop button pressed then testActive will be false from CheckButtons().
	if (testActive) {
		return 0;
	}
	return 1;
}

int VentSolenoid(int pin) {
	long start = millis();

	Serial.println("Solenoid 2 opening - venting");
	analogWrite(pin, 255);  // open solenoid 2

	// Check for button presses while holding sol 2 open
	while ((millis() - start) < sol2Dwell) {
		CheckButtons();
		LCD_Button_Action();
	}
	// Close solenoid 2
	analogWrite(pin, 0);
	Serial.println("Solenoid 2 closed");

	// If stop button pressed then testActive will be false from CheckButtons().
	if (testActive) {
		return 0;
	}
	return 1;

}

void StopMotor(int maxSpeed) {
	for (int i = maxSpeed; i > 0; i--) {
		myMotor->setSpeed(i);
		delay(1);
	}
	myMotor->run(RELEASE);
}

int RaiseMotorToUpperLim() {
	// start motor
	AccelerateMotor(BACKWARD);
	//AccelerateMotorBackward(maxSpeed);  // was AccelerateMotorForward
	while (upperLimitReached == false && testActive) {
		CheckButtons();
		// Below may cause blocking if one of the LCD buttons is pressed and 
		// could crash motor.
		LCD_Button_Action();
		//maxSpeed = GetMotorSpeed();
		//myMotor->setSpeed(maxSpeed);*/
	}
	// upper limit switch has been reached slow motor to stop
	StopMotor(maxSpeed);
	// if stop button hit cancel remainder of test -- should insert a safe shutdown call here
	if (testActive == false) {
		upperLimitReached = false;
		SafelyPauseExecution();
		return 1;
	}
	//reset limit
	upperLimitReached = false;
	return 0;
}

int LowerMotorToLowerLim() {
	//AccelerateMotorForward(maxSpeed); // was backward
	AccelerateMotor(FORWARD);
	while (lowerLimitReached == false && testActive) {
		CheckButtons();
		//This call could cause blocking and crash motor (fixed 8/12 to stop motor when opening interactive lcd)
		LCD_Button_Action();
		/*maxSpeed = GetMotorSpeed();
		myMotor->setSpeed(maxSpeed);*/
	}
	// lower limit switch has been reached slow motor to stop
	StopMotor(maxSpeed);
	// if stop button hit cancel remainder of test -- should insert a safe shutdown call here
	if (testActive == false) {
		lowerLimitReached = false;
		return 1;
	}
	// Reset alarm
	lowerLimitReached = false;
	return 0;
}

int GetMotorSpeed() {
	return analogRead(analogSpeedPin) / 4;
}

void PauseTest() {
	lcd.setCursor(0, 0);
	lcd.print("TEST PAUSED...  ");
	lcd.setCursor(0, 1);
	lcd.print("GrnBtn 2 resume");
	SafelyPauseExecution();

	// wait for button press
	while (testPaused == 1)
	{
		CheckButtons();
		LCD_Button_Action();
	}

	if (testPaused > 1) {
		lcd.setCursor(0, 0);
		lcd.print("TEST STOPPING...");
		lcd.setCursor(0, 1);
		lcd.print("                ");
		delay(2000);

	}
	else {
		lcd.setCursor(0, 0);
		lcd.print("Resuming Test...");
		lcd.setCursor(0, 1);
		lcd.print("                ");
		SafelyResumeExecution();
		delay(2000);
		startTime = resetStartTime(elapsedTimePausedTest);
	}
	SetDefaultLCDFormat();
}

void SafelyPauseExecution() {
	switch (state)
	{
	case RAISE_MOTOR:
		StopMotor(maxSpeed);
		VentSolenoid(analogSol2Pin);
		break;
	case LOWER_MOTOR:
		StopMotor(maxSpeed);
		VentSolenoid(analogSol2Pin);
		break;
	case PRESSURIZE:
		//close solenoid and vent
		analogWrite(analogSol1Pin, 0);
		VentSolenoid(analogSol2Pin);
		break;
	case VENT:
		VentSolenoid(analogSol2Pin);
		break;
	default:
		StopMotor(maxSpeed);
		VentSolenoid(analogSol2Pin);
	}
}

void SafelyResumeExecution() {
	switch (state)
	{
	case RAISE_MOTOR:
		// AccelerateMotorBackward(maxSpeed);		// was accelerateMotorForward
		AccelerateMotor(BACKWARD);
		break;
	case LOWER_MOTOR:
		// AccelerateMotorForward(maxSpeed);
		AccelerateMotor(FORWARD);
		break;
	case PRESSURIZE:
		// Pressurize Solenoid
		PressurizeSolenoid(analogSol1Pin);
		break;
	case VENT:
		// Vent solenoid
		VentSolenoid(analogSol2Pin);
		break;
	}
}

// triggered by a down button press from LCD_Button_Action()
// still working on this - 07/31/19
void InteractiveLCD() {
	if (testActive) {
		SafelyPauseExecution();
	}
	int i = 0;
	int n = sizeof(settings) / sizeof(settings[0]);
	//int n = 9;
	//Serial.println(n);
	bool breakOut = false;

	Serial.println(settings[i].title);
	lcd.setCursor(0, 0);
	lcd.print(settings[i].title);
	lcd.setCursor(0, 1);
	lcd.print("                ");
	lcd.setCursor(11, 1);
	lcd.print(settings[i].value);

	int lcd_key;
	//int voltage = 0;
	while (breakOut == false) {
		lcd_key = read_LCD_buttons();
		while (lcd_key == btnNONE) {
			lcd_key = read_LCD_buttons();
			/*voltage = analogRead(0);
			lcd.setCursor(0, 1);
			char str[9];
			snprintf(str, 9, "%4d v", voltage);
			lcd.print(str);*/

		}
		Serial.print("in loop, lcd_key = "); Serial.print(lcd_key);
		//Serial.print("; "); Serial.print(voltage); Serial.println(" v");
		switch (lcd_key)
		{
		case btnRIGHT:
			lcd.setCursor(11, 1);
			lcd.print("     ");
			lcd.setCursor(11, 1);
			if (settings[i].isStd) {
				if ((settings[i].value + settings[i].step) < settings[i].max) {
					settings[i].value = settings[i].value + settings[i].step;
					lcd.print(settings[i].value);
				}
				else {
					lcd.print("MAX ");
					delay(500);
					lcd.setCursor(11, 1);
					lcd.print("     ");
					lcd.setCursor(11, 1);
					lcd.print(settings[i].value);
				}
			}
			else if (i == RESET_COUNTER) {
				cycleCount++;
				cycleCountPrev++;
				lcd.setCursor(11, 1);
				lcd.print("     ");
				lcd.setCursor(11, 1);
				lcd.print(cycleCount);
			}
			break;

		case btnLEFT:
			lcd.setCursor(11, 1);
			lcd.print("     ");
			lcd.setCursor(11, 1);
			if (settings[i].isStd) {
				if ((settings[i].value - settings[i].step) > settings[i].min) {
					settings[i].value = settings[i].value - settings[i].step;
					lcd.print(settings[i].value);
				}
				else {
					lcd.print("MAX ");
					delay(500);
					lcd.setCursor(11, 1);
					lcd.print("     ");
					lcd.setCursor(11, 1);
					lcd.print(settings[i].value);
				}
			}
			else if (i == RESET_COUNTER) {
				cycleCount--;
				cycleCountPrev--;
				lcd.setCursor(11, 1);
				lcd.print("     ");
				lcd.setCursor(11, 1);
				lcd.print(cycleCount);
			}
			break;

		case btnDOWN:
			// increment position
			if (i < (n - 1)) {
				i++;
			}
			else {
				i = 0;
			}
			// print settings
			lcd.setCursor(0, 0);
			lcd.print("                ");
			lcd.setCursor(0, 0);
			lcd.print(settings[i].title);
			lcd.setCursor(0, 1);
			lcd.print("                ");
			lcd.setCursor(11, 1);

			switch (i) {
			case RESET_COUNTER:
				lcd.print(cycleCount);
				break;
			case HOME:
				break;
			default:
				lcd.print(settings[i].value);
			}
			break;

		case btnUP:
			// decrement position
			if (i == 0) {
				i = n - 1;
			}
			else {
				i--;
			}

			// print settings
			lcd.setCursor(0, 0);
			lcd.print("                ");
			lcd.setCursor(0, 0);
			lcd.print(settings[i].title);
			lcd.setCursor(0, 1);
			lcd.print("                ");
			lcd.setCursor(11, 1);

			switch (i) {
			case RESET_COUNTER:
				lcd.print(cycleCount);
				break;
			case HOME:
				break;
			default:
				lcd.print(settings[i].value);
			}
			break;

		case btnSELECT:
			switch (i)
			{
			case HOME:
				breakOut = true;
				SetDefaultLCDFormat();
				break;

			case MOTOR_SPEED:
				maxSpeed = settings[i].value;
				PrintSaveMsg();
				break;

			case UPPER_DWELL:
				upperLimDwell = settings[i].value;
				PrintSaveMsg();
				break;

			case LOWER_DWELL:
				lowerLimDwell = settings[i].value;
				PrintSaveMsg();
				break;

			case SOL1_DWELL:
				sol1Dwell = settings[i].value;
				PrintSaveMsg();
				break;

			case SOL2_DWELL:
				sol2Dwell = settings[i].value;
				PrintSaveMsg();
				break;

			case TGL_SPEED:
				toggleSpeed = settings[i].value;
				PrintSaveMsg();
				break;

			case RESET_COUNTER:
				cycleCount = 0;
				cycleCountPrev = -1;
				lcd.setCursor(11, 1);
				lcd.print("RESET");
				delay(1000);
				lcd.setCursor(11, 1);
				lcd.print("     ");
				lcd.setCursor(11, 1);
				lcd.print(cycleCount);
				break;

			case MAX_CYCLES:
				maxCycles = settings[i].value;
				PrintSaveMsg();
				break;

			default:
				break;
			}


		}
	}






}
