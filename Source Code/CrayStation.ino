#define VERSION "CMHASD V0.251104"
// Started code on 13 Oct 2024.  For Arduino Nano.
#pragma region GLOBAL SETTINGS
#include <Servo.h>
#include <EEPROM.h>
#include <Adafruit_PCF8575.h>               // For expansion board      0x20  https://github.com/adafruit/Adafruit_PCF8574
#include <Adafruit_BME280.h>                // For environment board    0x76  https://github.com/adafruit/Adafruit_BME280_Library
#include <LiquidCrystal_I2C.h>              // For handbox LCD display  0x27  https://github.com/johnrickman/LiquidCrystal_I2C
#include <DS3231M.h>                        // For RTC clock            0x68  https://github.com/Zanduino/DS3231M/tree/master
#include <SoftwareSerial.h>
//#include <wire.h>
//#include <TimerOne.h>
//#include <Wire.h>
//#include <Adafruit_Sensor.h>

#define P_OP_SW         2
#define P_CL_SW         3
#define P_RPI_PWR       4         // Directly controls power to RPi
#define P_MTR_PWM       5
#define P_MTR_DIR       6
#define P_SRV_PWR       7
#define P_TEL_PWR       8         // Power for charging scope
#define P_SPARE_D9      9
#define P_CAM_HTR       10        // Dew heater power for the all-sky camera  
#define P_SRV_SIG       11
#define P_RAIN          12
#define P_LED           13        // Power for the enclosure LEDs
#define P_SPARE_A0      A0
#define P_BUZZER        A1
#define P_AMPS          A2
#define P_HBX_CXN       A3
#define P_SDA           A4
#define P_SCL           A5
#define P_SOLAR         A6
#define BATT_V          A7

#define E_BUZZER        11        // Handbox buzzer pin on the PCF8785 expansion board in HandBox
#define E_KEY_R         6         // Right key pin
#define E_KEY_L         5         // Left key pin
#define E_KEY_U         4         // Up key pin
#define E_KEY_D         3         // Down key pin
#define E_KEY_ENTER     2         // Enter key pin
#define E_KEY_BACK      1         // Back key pin
#define E_KEY_NONE      0         // Value to return in a function when no key is being pressed

#define SCALE_AMPS      100       // 5A module = 185, 20A = 100, 30A = 66
#define SCALE_BATT      1         // Calibrated value to allow Arduino to measure battery voltage
#define SCALE_SOLAR     1         // Calibrated value to allow Arduino to measure solar panel voltage
#define BATT_BLACK      12.10     // Battery is less than 50% - needs charging.  System can't be used CRITICAL!!
#define BATT_RED        12.24     // Limited access to the RPi
#define BATT_AMBER      12.37     // Above this - all systems go! Below this - system should be kept in power saving mode
#define BATT_GREEN      12.40     // Above this when charging - back in GOOD status
#define LEVEL_BLACK     0         // All systems off - concentrate on charging battery
#define LEVEL_RED       1         // Power up RPi on the hour.  Turn off all unnecessary devices
#define LEVEL_AMBER     2         // Power up RPi every half hour.  Ensure lid closed and telescope power is off
#define LEVEL_GREEN     3         // All systems are go!

#define LCD_WIDTH     16          // Number of characters on each line of the LCD display
#define LCD_HEIGHT    2           // Number of lines on the LCD display
#define CANNOT_DO     '$'         // Response if a command can't be carried out.
#define UNKNOWN_CMD   '?'         // Response if an invalid command has been given

Servo myServo;                            // Servo for pushing telescope's power button
Adafruit_PCF8575 handBox;                 // Access handbox's keys and buzzer
Adafruit_BME280 bme;                      // Thermometer, barometer, hygrometer (humidity)
LiquidCrystal_I2C lcd(0x27, 16, 2);       // I2C address 0x27, 16 characters per row, 2 rows
DS3231M_Class RTC;                        // On baord Real Time Clock (RTC) and thermometer
DateTime now;                             // Class for getting and writing data from the RTC
SoftwareSerial GPSserial(P_SPARE_D9, 9);  // For receiving NEMA sentences from a GPS module

volatile unsigned long timerLid = 0;      // Timer for lid timeout
unsigned long timerPowerOff = 0;          // For RPi power
unsigned long timerPowerOn = 0;           // For RPi power
unsigned long timerCloseLid = 0;          // Timer for when lid must close
unsigned long tick =0;                    // Increments once a second - for timing functions
unsigned long timerHandBox = 0;           // Timer for before resetting handbox when disconnected
unsigned long timerServo = 0;             // Timer for pressing the telescope's on/off button
unsigned long timerBattCharging = 0;      // Timer for charging battery
unsigned long ardClock = -1;              // Clock which increments once a second (rolls over after 136 years!)
long secsPastHour = 3600;                 // RPi power timer settings

char c;                                   // Used to get a command from the RPi and to a one character response
volatile byte pwm = 0;                    // Motor stopped
byte dir = 1;                             // Motor direction for closing.
byte heater = 0;                          // SkyCam Heater
byte battLevel = LEVEL_GREEN;             // Assume battery is good at start.

byte x = 0;                               // LCD cursor coords
byte y = 0;
bool handBoxHere = false;                 // Last known connection status of handbox
byte lastKey = E_KEY_NONE;                // Last handbox key pressed
byte pressedKey = E_KEY_NONE;             // Current key being pressed
bool lcdBacklight, lcdDisplay, lcdBlink, lcdCursor; // Last known LCD settings
char disp0[17];                           // Display buffer for top line
char disp1[17];                           // Display buffer for bottom line
char pwd[64];                             // Maximum length of a WiFi password is 63 chars
bool gettingPwd = false;                  // Indicates whether HandBox is getting password from user
byte handBoxInUse = 10;                   // If RPi is not using the RPi then HandBox displays the clock
char GPSsentence[2][82];
byte GPSlinePos = 0;
bool GPSline = 0;

bool chargingScope = false;               // Telescope charging status
bool servoReturning = false;              // If true, the servo has been set in motion
bool poweringDown = false;                // RPi is powering down, because it said it is or if the battery is at critical level
bool ignoreRain = false;                  // For Superuser use.  Making this true allows roof to be opened regardless of the weather

struct eeprom {
  int reset;                        // If reset = 255 then the EEPROM is factory resetted to these settings
  int servoHome = 90;               // Angle for non-contact position of servo arm
  int servoPush = 147;              // Angle for contact position of servo arm
  unsigned long pulsePeriod = 600;  // Give the RPi, say, 60 secs to boot-up before seeing if it's responding
  unsigned long lidTimeOut = 10000; // Give opening/closing lid, say, 30 secs to carry out operation.
  long secsStayAwake = 600;         // RPi power timer settings
} ee;

struct clock {                      // Structure to keep tabs on the Arduino's clock.
  int yr = 2025;                    // This 'clock' is synched to the RTC every minute.
  byte mth = 11;                    // This allows the Arduino to run as a back-up clock if the RTC fails
  byte day = 2;
  byte hr = 0;
  byte min = 0;
  unsigned long sec = 0;
} clock;

#pragma endregion
#pragma region SETUP & LOOP
//=================================================================================================
// SETUP & LOOP
//=================================================================================================
void setup() {
  // Intialise pins
  pinMode(P_MTR_DIR,        OUTPUT);            // Define digital pins
  pinMode(P_MTR_PWM,        OUTPUT);
  pinMode(P_SRV_PWR,        OUTPUT);
  pinMode(P_BUZZER,         OUTPUT);
  pinMode(P_RPI_PWR,        OUTPUT);
  pinMode(P_TEL_PWR,        OUTPUT);
  pinMode(P_LED,            OUTPUT);
  pinMode(P_RAIN,           INPUT_PULLUP);
  pinMode(P_OP_SW,          INPUT_PULLUP);
  pinMode(P_CL_SW,          INPUT_PULLUP);
  pinMode(P_HBX_CXN,        INPUT_PULLUP);
  pinMode(P_SOLAR,          INPUT);
  pinMode(BATT_V,           INPUT);
  pinMode(P_AMPS,           INPUT);
  pinMode(P_SPARE_A0,       OUTPUT);            // Power to GPS Module

  analogWrite(P_MTR_PWM,    LOW);               // No power to the motor
  digitalWrite(P_MTR_DIR,   LOW);               // Default to 'Open'
  digitalWrite(P_SRV_PWR,   LOW);
  digitalWrite(P_BUZZER,    LOW);
  digitalWrite(P_CAM_HTR,   LOW);
  digitalWrite(P_TEL_PWR,   LOW);
  digitalWrite(P_RPI_PWR,   HIGH);              // Power-up the Raspberry Pi
  digitalWrite(P_SPARE_A0,  LOW);               // No power to GPS module

  // Intialise Settings & Clock
  int addr;
  EEPROM.get(0, addr);                          // Get EEPROM address where settings are held
  if (addr > 1022) addr = 2;
  EEPROM.get(addr, ee);
  EEPROM.get(1014, clock);                      // Asssume what is stored here is the correct time
  syncToRTC();
  setArdClock();
  GPSsentence[0][0] = GPSsentence[1][0] = '\0'; // Make ready to receive first output from GPS module

  // Serial port
  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println("\nInitialising...");
  GPSserial.begin(9600);
  beep(500);

 // Initialise handbox
  checkHandBox();

  // Lid limit switches interrupts
  //attachInterrupt(digitalPinToInterrupt(P_OP_SW), stopMotor, LOW);
  //attachInterrupt(digitalPinToInterrupt(P_CL_SW), stopMotor, LOW);

  // Initialise telescope power button servo
  myServo.attach(P_SRV_SIG);
  myServo.write(ee.servoHome);

  Serial.println("Ready");
}
//=================================================================================================
void loop() {
  tickTock();                           // Update clocks

  if (ee.reset == 255) {
    // Reset EEPROM with default values if ee.reset is not 0
    for (int i = 2; i < 1014; i++) EEPROM.put(i, 255);
    ee.reset = 0;
    ee.servoHome = 90;
    ee.servoPush = 147;
    ee.lidTimeOut = 10000;
    ee.secsStayAwake = 600;
    EEPROM.put(1014, clock);
  }
  updateEEPROM();
  powerManagement();
  checkRain();                          // Check if raining - close box if it is
  serviceMotor();                       // Control motor and monitor enclosure switches
  commands();                           // Service Comms from RPi
  checkHandBox();
  serviceServo();
  serviceGPS();
}
//=================================================================================================
#pragma endregion
#pragma region COMMANDS
//=================================================================================================
// COMMANDS
//=================================================================================================
void commands(){
  if (!Serial.available()) return;              // Return if nothing received
  c = Serial.read();
  beep(1);

  switch (c){
    case 'O': openLid();                break;
    case 'C': closeLid();               break;
    case 'm': getMotorStatus();         break;  // c = closed, c = closing, o = open, O = opening, T = timed out
    case 'M': setMotorTimeOut();        break;  // Set lid opening/closing time out
    case 'X': stopMotor();              break;
    case 'c': setCloseTimer();          break;
    case '=': nudgeLid();               break;  // **CAREFUL** Force moves the lid for x ms in either direction, regardless.

    case 'B': telescopeButton();        break;  // Press telescope's on/off button for supplied ms
    case '/': setPushAngle();           break;  // Set/retrieve servo's arm's action angle
    case '|': setHomeAngle();           break;  // Set/retrieve servo's arm's home angle
    case '^': chargeScope();            break;  // Turn on/off/get(-1) power for charging scope

    case 'L': leds();                   break;  // Turn on/off/get power to the LEDs
    case 'D': dewHeater();              break;  // Turn on/off/get(-1) power to the skycam's dew heater
    case 'Z': soundBuzzer();            break;  // Sound buzzer (& handbox buzzer) for a given period

    case '#': checkHandBox();           break;  // Sees if the handbox is connected, and if Nano will be powering off the RPi
    case 'k': getKey();                 break;  // Returns pressed key info on handbox
    case '[': printText();              break;  // Sent a string of text to the HandBox display
    case '@': setCursor();              break;  // Set coordinates of the Handbox display's cursor
    case '$': escapeCodes();            break;  // Set HandBox diplay's settings
    case 'E': enterPassword();          break;  // Get HandBox to get a WiFi password from the user
    case 'P': getPassword();            break;  // Returns the password supplied by the user
    case 'g': c = '0' + gettingPwd;     break;  // See if a password is being got by the handbox

    case '~': stillActive();            break;  // RPi will still keep sending this command
    case 'z': goToSleep();              break;  // Does RPi need to go to sleep? (E.g. low battery)
    case 'G': goodNight();              break;  // Cut power to RPi in 1 minute
    case 'T': setClock();               break;  // Set Arduino's clock and RTC YYYY/MM/DD HH:MM:SS
    case 'A': getArdTime();             break;  // GEt Arduino clock's time and date
    case 'R': getRTCtime();             break;  // Get RTC's time and date
    case 'p': powerOffTimer();          break;  // Time left in seconds before RPi is turned off (don't extend timerPowerOff)
    case 'S': digitalWrite(P_SPARE_A0, HIGH);           break;  // GPS power ON
    case 's': digitalWrite(P_SPARE_A0, LOW);            break;  // GPS power OFF
    case '*': Serial.println(GPSsentence[!GPSline]);    break;  // Send last line of data received from the GPS module

    case 'r': checkRain();              break;  // Is the sensor indicating that it is raining? '1' = yes, '0' = no
    case 'F': ignoreRain = true;        break;  // Ignore rain sensor so that the lid can be opened and closed even if raining
    case 'f': ignoreRain = false;       break;  // Start recognising rain sensor
    case 't': getTemperature();         break;  // Get the BME280 sensor's temperature in Celsius
    case 'h': getHumidity();            break;  // Get the BME280 sensor's hygrometer reading as a %age
    case 'b': getBarometer();           break;  // Get the BME280 sensor's air pressure in mb
    
    case 'v': getBatteryVolts();        break;  // Get the battery's voltage
    case 'u': getSolarVolts();          break;  // Get the solar panel's voltage
    case 'a': getAmps();                break;  // 12V battery amps currently being consumed

    case 'e': ee.reset = 255;           break;  // Resets the EERPOM contents in the Loop function above
    case 'i': getVersion();             break;  // Get version of this code
    case '_': restart();                break;  // Restart the Arduino
    case '?': systemStatus();           break;  // Full diagnostic report
    case '\n':  c = '\0';               break;
    case '\r':  c = '\0';               break;
    default:    c = UNKNOWN_CMD;        break;  // Unknown command
  }
  if (c != '\0') Serial.println(c);
}
//-------------------------------------------------------------------------------------------------
void printText(){
  if (gettingPwd) {
    c = CANNOT_DO;
    return;
  }
  unsigned long timer = millis() + 1000;
  char chr[2];
  while (timer > millis()){
    Serial.readBytes(chr, 1);
    switch (chr[0]) {
      case '\0':  return;             break;
      case '\r':  return;             break;
      case '\n':  return;             break;
      case ']':   return;             break;
      case '@':   setCursor();        break;
      case '$':   escapeCodes();      break;
      default:    printChar(chr[0]);  break;
    }
  }
}
//-------------------------------------------------------------------------------------------------
void escapeCodes(){
// "$" is used as the escape character
  if (gettingPwd) {
    c = CANNOT_DO;
    return;
  }

  char chr[2];
  Serial.readBytes(chr, 1);
  switch (chr[0]) {
    case 'I': lcd.init();                           break;
    case 'L': lcd.backlight();    lcdBacklight = 1; break;
    case 'l': lcd.noBacklight();  lcdBacklight = 0; break;
    case 'D': lcd.display();      lcdDisplay = 1;   break;
    case 'd': lcd.noDisplay();    lcdDisplay = 0;   break;
    case 'B': lcd.blink();        lcdBlink = 1;     break;
    case 'b': lcd.noBlink();      lcdBlink = 0;     break;
    case 'C': lcd.cursor();       lcdCursor = 1;    break;
    case 'c': lcd.noCursor();     lcdCursor = 0;    break;
    case '[': printChar('[');                       break;  // Print characters used as escape codes
    case ']': printChar(']');                       break;
    case '@': printChar('@');                       break;
    case '$': printChar('$');                       break;
    case 'H': lcd.setCursor(x = 0, y = 0);          break;  // Cursor home
    case 'N': lcd.setCursor(x = 0, y = 1 - y);      break;  // Move cursor onto beginning of the other row
    case 'R': lcd.setCursor(x = 0, y);              break;  // Return cursor to start of row
    case 'X': clearLCD();                           break;  // Clear screen & return cursor home
  }
}
//=================================================================================================
#pragma endregion
#pragma region POWER MANAGEMENT
//=================================================================================================
// POWER MANAGEMENT
//=================================================================================================
void powerManagement() {
  unsigned long beepTimer = millis();

  serviceRPiTimers();
  switch (batteryStatus()) {
    case LEVEL_BLACK:
      if (battLevel == LEVEL_BLACK) return;
      digitalWrite(P_CAM_HTR, LOW);                               // Turn off unnecessary devices   
      digitalWrite(P_TEL_PWR, LOW);
      digitalWrite(P_LED, LOW);
      poweringDown = true;                                        // Power off the RPI in 60 secs to hopefully
      timerPowerOff = ardClock + 60;                              // allowing time for the RPi to close gracefully
      handboxChargingStatus();
      closeLid();
      if (millis() > beepTimer) digitalWrite(P_BUZZER, HIGH);       // Start buzzer - allow it to sound for 100ms
      if (millis() > beepTimer + 100) digitalWrite(P_BUZZER, LOW);  // Turn off buzzer after 100ms
      if (millis() > beepTimer + 10000) beepTimer = millis();       // Reset beepTimer after 10secs
      secsPastHour = 3600;
      break;

    case LEVEL_RED:
      if (battLevel < LEVEL_RED) return;
      digitalWrite(P_CAM_HTR, LOW);                               // Switch off unnecessary devices
      digitalWrite(P_TEL_PWR, LOW);
      digitalWrite(P_LED, LOW);
      closeLid();
      secsPastHour = 3600;
      break;

    case LEVEL_AMBER:
      if (battLevel < LEVEL_AMBER) return;
      if (ardClock > timerPowerOff) {                             // Time to sleep?
        digitalWrite(P_CAM_HTR, LOW);                             // Put devices to sleep
        digitalWrite(P_TEL_PWR, LOW);
        digitalWrite(P_LED, LOW);
        secsPastHour = 3600;
      }
      break;

    case LEVEL_GREEN:
      if (battLevel < LEVEL_GREEN) return;
      battLevel = LEVEL_GREEN;
        digitalWrite(P_TEL_PWR, HIGH);
        secsPastHour = 1800;
      break;
  }
} 
//-------------------------------------------------------------------------------------------------
byte batteryStatus() {
  byte currLevel = LEVEL_BLACK;
  float v = measureVolts(BATT_V, SCALE_BATT, 20);

  if (v >= BATT_RED)   currLevel = LEVEL_RED;
  if (v >= BATT_AMBER) currLevel = LEVEL_AMBER;
  if (v >= BATT_GREEN) currLevel = LEVEL_GREEN;

  if (currLevel < battLevel || ardClock > timerBattCharging) {
    battLevel = currLevel;
    timerBattCharging = ardClock + 600;
  } 
  return battLevel;
}
//-------------------------------------------------------------------------------------------------
void leds(){                                // L
  delay(10);
  switch (Serial.read()) {
    case '0': 
      digitalWrite(P_LED, LOW);
      break;
    case '1':
      if (batteryStatus()) {
        digitalWrite(P_LED, HIGH);
        c = CANNOT_DO;
      }
      break;
    case '\n':
      Serial.println(digitalRead(P_LED));
      c = '\0';
      break;
  }
}
//-------------------------------------------------------------------------------------------------
void dewHeater(){                           // Dx
  int value = Serial.parseInt();
  if (value >= 0 || value <= 100) {
      if (batteryStatus() < LEVEL_GREEN) value = 0;
      analogWrite(P_CAM_HTR, heater = (value * 255) / 100);
  }
  Serial.println(value);
  c = '\0';
}
// 210ma at 12V needs to be delivered (maximum) for the All-Sky Cam dome.
// dewcontrol.com heater uses 57 x 3300 ohm (332) resistors in paralled on a ring which is powered by 12V.
// Total resistance = 57.9 ohms. A fully charged 12V 7ah would last 33 hours.
//-------------------------------------------------------------------------------------------------
void getBatteryVolts() {                    // v
  float volts = measureVolts(BATT_V, SCALE_BATT, 20);
  c = '\0';
  if (volts >= 14.0) volts = 13.9;
  Serial.print(volts, 1);                                     // Needs to be in the form like 13.7V (Good)
  if(volts < 12.1) Serial.println("V (Bad) ");
  else if (volts < 12.6) Serial.println("V (Poor)");
  else Serial.println("V (Good)");
}
//-------------------------------------------------------------------------------------------------
void getSolarVolts() {                      // u
  float volts = measureVolts(P_SOLAR, SCALE_SOLAR, 20);
  c = '\0';
  if (volts > 30) volts = 30.0;
  Serial.print(volts, 1);                                     // Needs to be in the form like 27.7V
  Serial.println('V');
}
//-------------------------------------------------------------------------------------------------
float measureVolts(int pin, float scale, int samples) {
  float average = 0;
  for (byte i = 0; i < samples; i++){
    average += analogRead(pin);
    delay(1);
  }
  return average / samples * scale;
}
//-------------------------------------------------------------------------------------------------
void getAmps() {                            // a
  // https://www.amazon.co.uk/s?k=ACS712+20A&crid=3SKQ3JVK7WWHD&sprefix=acs712+20a%2Caps%2C63&ref=nb_sb_noss
  // AZDelivery ACS712 20A
  // https://cdn.shopify.com/s/files/1/1509/1638/files/AZ043_A_8-6_EN_B0736DYV3W_c2659f11-4c11-4aaf-8f7e-3d84ee0c3b9a.pdf?v=1721208630
  // SCALE_AMPS = 5A module = 185, 20A = 100, 30A = 66

  float average = 0;                                  // Take an average of 100 readings to get a stable reading
  for (byte i = 0; i < 100; i++) {
    average += analogRead(P_AMPS);
    delay(1);
  }
  average /= 100;
  float voltage = (average / 1023.0) * 5000.0;
  float current = (voltage - 2500.0) / SCALE_AMPS;    // 2500 may need to be changed after calibration for 0 amps
  if (current < 0) current = 0;

  c = '\0';
  Serial.print(current * 1000.0);                     // Return milli-amps
  Serial.println("mA");
}
//=================================================================================================
#pragma endregion
#pragma region TELESCOPE
//=================================================================================================
// TELESCOPE
//=================================================================================================
void telescopeButton(){                     // Bx (x is in  ms)
  int pressTime = Serial.parseInt();        // Get how long to press the telesope power button for in ms.
  if (pressTime > 1){
    digitalWrite(P_SRV_PWR, HIGH);          // Power up the servo
    myServo.write(ee.servoPush);            // Press the power button
    timerServo = millis() + pressTime;
  }
  else c = CANNOT_DO;                       // Unable to carry out operation
}
//-------------------------------------------------------------------------------------------------
void serviceServo() {
  if (timerServo && millis() > timerServo) {// Test to see if timerServo was set and has expired
    if (servoReturning) {
      timerServo = 0;                       // If servo was returning the reset timer
      digitalWrite(P_SRV_PWR, LOW);         // and cut power to the servo.
      servoReturning = false;               // Servo is no longer moving
    }
    else {
      myServo.write(ee.servoHome);          // Tell servo to return to home position so releasing scope's power button
      servoReturning = true;                // Servo arm is moving
      timerServo = millis() + 1000;         // Allow 1000ms for the servo arm to return before powering off 
    }
  }
}
//-------------------------------------------------------------------------------------------------
void chargeScope(){                         // Ex (no parameter given returns current state)
  delay(10);
  switch (Serial.read()){
    case '0':
      digitalWrite(P_TEL_PWR, chargingScope = 0);   // Stop charging
      break;
    case '1':
      digitalWrite(P_TEL_PWR, chargingScope = 1);   // Start charging
      break;
    case '\n':
      c = chargingScope + '0';                      // Charging status (on or off)
      break;
  }
}
//-------------------------------------------------------------------------------------------------
void setPushAngle(){                        // /
  ee.servoPush = getAngle(ee.servoPush);    // Set servo angle for pressing the scope's on/off button
}
//-------------------------------------------------------------------------------------------------
void setHomeAngle(){                        /* | */
  ee.servoHome = getAngle(ee.servoHome);    // Set servo angle for home position, i.e. not pressing the button
}
//-------------------------------------------------------------------------------------------------
int getAngle(int angle){
  int a = Serial.parseInt();
  if (a > 180 || a < 0) angle = a;
  Serial.println(angle);
  c = '\0';
  return angle;
}
//=================================================================================================
#pragma endregion
#pragma region CLOCKS AND TIMERS
//=================================================================================================
// CLOCKS AND TIMERS
//=================================================================================================
void serviceRPiTimers() {
  if (ardClock > timerPowerOn && ardClock < timerPowerOff) {
    digitalWrite(P_RPI_PWR, HIGH);                                  // Good morning RPi
    setRPiTimers();
  }
  if (ardClock > timerPowerOff) {
    digitalWrite(P_RPI_PWR, LOW);                                   // Good night RPi
    digitalWrite(P_LED, LOW);
    digitalWrite(P_CAM_HTR, LOW);                                   // (Should this be done?)
    setRPiTimers();
  }
}
// ------------------------------------------------------------------------------------------------
void setRPiTimers() {
    timerPowerOn  = (unsigned long)(ardClock / secsPastHour) * secsPastHour + secsPastHour; 
    timerPowerOff = timerPowerOn + ee.secsStayAwake;
}
// ------------------------------------------------------------------------------------------------
void setCloseTimer() {
  if (unsigned long val = Serial.parseInt() > 0) timerCloseLid = val;
  c = '\0';
  Serial.println(timerCloseLid);
}
//-------------------------------------------------------------------------------------------------
void stillActive() {                        // ~
  if (!poweringDown) timerPowerOff = ardClock + ee.secsStayAwake; // RPi says it wants to stay awake - postpone time to turn off
  else c = CANNOT_DO;                                             // Return error code if already powering down
}
//-------------------------------------------------------------------------------------------------
void goToSleep() {
  timerPowerOff = ardClock + 60;                        // RPi wants to go to sleep
  poweringDown = true;
}
//-------------------------------------------------------------------------------------------------
void goodNight() {
  c = '\0';
  if (batteryStatus() == LEVEL_BLACK && !poweringDown) {
    timerPowerOff = ardClock + 60;
    poweringDown = true;
    Serial.print('0');                                  // Print an extra leading 0 - the RPi WILL be powered off
  }
  Serial.println(timerPowerOff - ardClock);             // Time left until sleepy time
}
//-------------------------------------------------------------------------------------------------
void setArdClock() {    // NEEDS MORE WORK
  long timerPowerOnDiff = (timerPowerOn - ardClock);
  long timerPowerOffDiff = (timerPowerOff - ardClock);
  ardClock = (unsigned long)(ardClock / 3600) * 3600 + clock.min * 60 + clock.sec;      // Sync ardClock
  if (timerPowerOn > (unsigned long)timerPowerOnDiff)   timerPowerOn =  ardClock + timerPowerOnDiff;   // Amend timers
  if (timerPowerOff > (unsigned long)timerPowerOffDiff) timerPowerOff = ardClock + timerPowerOffDiff;
}
//-------------------------------------------------------------------------------------------------
void powerOffTimer(){                       // p
  Serial.println(timerPowerOff >0 ? timerPowerOff - ardClock : 0);  // Time left before RPi is powered off
  c = '\0';
}
//-------------------------------------------------------------------------------------------------
void setClock(){                            // Tyyyy/mm/dd hh:mm:ss
  int yr, mth, day, hr, min, sec;
  yr = mth = day = hr = min = sec = 0;

  yr  = Serial.parseInt();
  mth = Serial.parseInt();
  day = Serial.parseInt();
  hr  = Serial.parseInt();
  min = Serial.parseInt();
  sec = Serial.parseInt();
  if (!setClocks(yr, mth, day, hr, min, sec)) c = CANNOT_DO;
}
//-------------------------------------------------------------------------------------------------
bool setClocks(int yr, int mth, int day, int hr, int min, int sec) {
  if (yr  > 2099 || yr  < 2025) return false;
  if (mth > 12   || mth < 1)    return false;
  if (day > 31   || day < 1)    return false;
  if (hr  > 23   || hr  < 0)    return false;
  if (min > 59   || min < 0)    return false;
  if (sec > 59   || sec < 0)    return false;
  if (mth == 2) if (day > 29 || (day == 29 && yr % 4 > 0)) return false;

  if (RTC.begin()) {
    RTC.adjust(DateTime(yr, mth, day, hr, min, sec));
  }
  clock.yr = yr;
  clock.mth = mth;
  clock.day = day;
  clock.hr = hr;
  clock.min = min;
  clock.sec = sec;
  ardClock = min * 60 + sec;
  return true;
}
//-------------------------------------------------------------------------------------------------
void getArdTime(){                          // <
  char buf[20];
  sprintf(buf, "%04d/%02d/%02d %02d:%02d:%02d", clock.yr, clock.mth, clock.day, clock.hr, clock.min, int(clock.sec));
  Serial.println(buf);
  c = '\0';
}
//-------------------------------------------------------------------------------------------------
void getRTCtime(){                          // >
  char buf[20];
  DateTime now = RTC.now();
  sprintf(buf, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  Serial.println(buf);
  c = '\0';
}
//-------------------------------------------------------------------------------------------------
bool syncToRTC() {
  if(!RTC.begin()) return false;
  DateTime now = RTC.now();
  clock.yr = now.year();
  clock.mth = now.month();
  clock.day = now.day();
  clock.hr = now.hour();
  clock.min = now.minute();
  clock.sec = now.second();
  tick = millis() + 1000;
  setArdClock();
  return true;
}
//-------------------------------------------------------------------------------------------------
void handBoxUTC () {
  if (handBoxInUse > 2) { // Wait a bit before starting to show UTC time
    handBoxInUse--;
    return;
  }
  memcpy(disp0, "Waiting for RPi ", 16);
  sprintf(disp1, " UTC: %02d:%02d:%02d  ", clock.hr, clock.min, int(clock.sec));
  printLCDbuffer();
}
//-------------------------------------------------------------------------------------------------
void tickTock(){
  if (tick < 1000 && millis() > 0x7fffffff) return;         // Account for the ~49 day millis() overflow back to 0.
  if (millis() < tick) return;
  ardClock += (millis() - tick) / 1000 + 1;
  clock.sec += (millis() - tick) / 1000 + 1;
  tick += 1000;
  handBoxUTC();                                             // Display UTC on HandBox LCD.
  if (clock.sec < 60) return;
  if (syncToRTC()) return;                                  // If RTC is working, sync to it.
  if (measureVolts(BATT_V, SCALE_BATT, 20) < LEVEL_BLACK) { // Battery is in dire straits.
    beep(50);                                               // Cry for help
    delay(100);
    beep(50);
    delay(100);
    beep(50);
  }

  while (clock.sec > 59) {
    clock.sec -= 60;
    if (++clock.min < 60) return;
    clock.min -= 60;
    if (++clock.hr < 24) return; 
    clock.hr -= 24;
    if (++clock.day < 29) return;
    if (clock.mth == 2) if(clock.yr % 4 == 0 && clock.day == 29) return;
    if (clock.day == 30) if (clock.mth == 4 || clock.mth == 6 || clock.mth == 9 || clock.mth == 11) return;
    if (clock.day == 31) return;
    clock.day = 1;
    if (++clock.mth < 13) return;
    clock.mth = 1;
    clock.yr++;
  }
}
//=================================================================================================
#pragma endregion
#pragma region LOCAL CONDITIONS
//=================================================================================================
// LOCAL CONDITIONS
//=================================================================================================
void getTemperature(){                    // t
  // bme: https://cdn.shopify.com/s/files/1/1509/1638/files/AZ109_A3-7_EN_B07D8T4HP6_068a8db7-3572-4ef5-b396-be75b7ec8810.pdf?v=1721047611
  // RTC: https://github.com/Zanduino/DS3231M/tree/master
  c = '\0';
  if (bme.begin(0x76)) {
    printTemperature();
  }
  else if (RTC.begin()) {
    Serial.print(RTC.temperature() / 100.0, 1);
    Serial.print('C');
  }
  else Serial.println("99.9C");
}
//-------------------------------------------------------------------------------------------------
void printTemperature() {
  Serial.print(bme.readTemperature(), 1);
  Serial.println("C");
}
//-------------------------------------------------------------------------------------------------
void getHumidity(){                       // h
  // https://cdn.shopify.com/s/files/1/1509/1638/files/AZ109_A3-7_EN_B07D8T4HP6_068a8db7-3572-4ef5-b396-be75b7ec8810.pdf?v=1721047611
  c = '\0';
  if (bme.begin(0x76)) {
    printHumidity();
  }
  else {
    Serial.println("999%");
  }
}
//-------------------------------------------------------------------------------------------------
void printHumidity() {
  Serial.print(bme.readHumidity(), 1);
  Serial.println("%");  
}
//-------------------------------------------------------------------------------------------------
void getBarometer(){                      // b
  // https://cdn.shopify.com/s/files/1/1509/1638/files/AZ109_A3-7_EN_B07D8T4HP6_068a8db7-3572-4ef5-b396-be75b7ec8810.pdf?v=1721047611
  c = '\0';
  if (bme.begin(0x76)) {
    printPressure();
  }
  else {
    Serial.println("999.9mb");
  }
}
//-------------------------------------------------------------------------------------------------
void printPressure() {
  Serial.print(float(bme.readPressure()) / 100, 1);
  Serial.println("mb");
}
//-------------------------------------------------------------------------------------------------
bool checkRain(){                         // r
  // Use for Hydreon RG-9 rain sensor.  Intructions:
  // https://rainsensors.com/wp-content/uploads/sites/3/2022/03/2022.02.17-rev-1.200-rg-9_instructions.pdf

  c = '0' + !digitalRead(P_RAIN);                   // '0' = dry, '1' = raining
  if (ignoreRain) c = '0';                          // Allow lid to open even if rain (in case rain sensor has failed)
  return (c == '1');
}
//=================================================================================================
#pragma endregion
#pragma region LID CONTROL
//=================================================================================================
//  LID CONTROL
//=================================================================================================
void openLid(){
  if (checkRain()) {
    c = CANNOT_DO;
    return;
  }
  if(digitalRead(getCloseSw())) {                     // Only start motor if not open
    if (dir == 1) stopMotor();                        // Stop motor if lid is closing
    pwm = 255;                                        // Start ramping up motor
    dir = 0;                                          // Open lid
    timerLid = millis() + ee.lidTimeOut;              // Setup timeout
  }
}
//-------------------------------------------------------------------------------------------------
void closeLid(){
  if(digitalRead(getOpenSw())) {                      // Only start motor if not closed
    if (dir == 0) stopMotor();                        // Stop if motor is closing
    pwm = 255;                                        // Start ramping up motor
    dir = 1;                                          // Close lid
    timerLid = millis() + ee.lidTimeOut;              // Setup timeout
  }
}
//-------------------------------------------------------------------------------------------------
void getMotorStatus(){
  if (checkTimedOut()) Serial.print('T');              // 'T' means timed out (return "Tc" or "To")
  if (pwm > 0) c = dir ? 'C' : 'O';                   // If motor is moving then 'C' means closing and 'O' is opening.

  if (getOpenSw()) {
    c = 'o';
  }
  else if (getCloseSw()) {
    c = 'c';
  }
}
//-------------------------------------------------------------------------------------------------
bool getOpenSw() {
  checkTimedOut();
  return !digitalRead(P_OP_SW);
}
//-------------------------------------------------------------------------------------------------
bool getCloseSw() {
  checkTimedOut();
  return !digitalRead(P_CL_SW);
}
//-------------------------------------------------------------------------------------------------
bool checkTimedOut() {
  if (millis() < timerLid) return false;
  stopMotor();
  return true;
}
//-------------------------------------------------------------------------------------------------
void serviceMotor(){
  if (ardClock > timerCloseLid) {
    timerCloseLid = -1;
    closeLid();
    return;
  }
  if (checkTimedOut()) return;
  digitalWrite(P_MTR_PWM, dir);
  if (pwm) {
    if (dir) {
      detachInterrupt(digitalPinToInterrupt(P_OP_SW));                    // Reset interrupt for open switch
      attachInterrupt(digitalPinToInterrupt(P_CL_SW), stopMotor, LOW);    // Set up interrupt for closed switch
      if (!getCloseSw()){                                                 // Stop motor if closed
        stopMotor();
        return;
      }
    }
    else {
      detachInterrupt(digitalPinToInterrupt(P_CL_SW));                    // Reset interrupt for closed switch
      attachInterrupt(digitalPinToInterrupt(P_OP_SW), stopMotor, LOW);  // Set up interrupt for open switch
      if (!getOpenSw()){                                                  // Stop motor if open
        stopMotor();
        return;
      }
    }
  }
}
//-------------------------------------------------------------------------------------------------
void setMotorTimeOut(){                          // Mx
  delay(20);                                // Wait a little for next character
  switch (Serial.peek()) {
    case '\r':                              // No parameter given so the current servoAnlge will be return
      break;
    case '\n':
      break;
    case '\0':
      break;
    default:
      unsigned long getTimeOut = Serial.parseInt();
      if (getTimeOut > 60000 || getTimeOut < 0){
        c = CANNOT_DO;
        return;
      };
      ee.lidTimeOut = getTimeOut;
      Serial.print(ee.lidTimeOut);
    break;
  }
  c = '\0';
  return;
}
//-------------------------------------------------------------------------------------------------
void stopMotor(){                                // X     ***This is also used as an Interrupt Service Routine***
// Either the open or closed switched, or the motor button has been pressed
  noInterrupts();
  digitalWrite(P_MTR_PWM, pwm = 0);                       // Stop the motor, pwm = 0 means motor has stopped
  detachInterrupt(digitalPinToInterrupt(P_OP_SW));        // Reset interrupt for closed switch
  detachInterrupt(digitalPinToInterrupt(P_CL_SW));        // Reset interrupt for open switch
  interrupts();
  timerLid = 0;                                           // Stop time-out timer
}
//-------------------------------------------------------------------------------------------------
void nudgeLid() {                                // Nx     (-x = close lid for x ms, x = open lid for x ms)
  int value;
  if ((value = Serial.parseInt()) == 0) return;
  digitalWrite(P_MTR_DIR, value > 0 ? LOW: HIGH);
  if (value < 0) value = -value;
  noInterrupts();
  analogWrite(P_MTR_PWM, 255);
  delay(value);
  analogWrite(P_MTR_PWM, 0);
  interrupts();
}
//=================================================================================================
#pragma endregion
#pragma region HANDBOX
//=================================================================================================
// HANDBOX
//=================================================================================================
bool checkHandBox() {
  //c = '1';                                          // ***REMOVE WHEN GOING LIVE***
  //return handBoxHere = true;                        // ***REMOVE WHEN GOING LIVE***

  if (handBoxHere) timerHandBox = ardClock+ 300;      // Update disconnection timer (5 mins)
  else if (ardClock > timerHandBox) timerHandBox = 0; // If disconnect for more than 5 mins the handbox is reset

  if (digitalRead(P_HBX_CXN) == handBoxHere) {        // Connection status changed?
    handBoxHere = false;                              // Assume connection is lost
    if (!digitalRead(P_HBX_CXN)){;                    // See if connected
      delay (250);                                    // Allow time for the hand controller to be connected
      if (!digitalRead(P_HBX_CXN)) {                  // Handbox attached?
        handBoxHere = true;                           // Yep, it is
        connectHandBox();                             // Intialise handbox
      }
    }
  }
  c = handBoxHere + '0';
  c += poweringDown * 2;
  return handBoxHere;
}
//-------------------------------------------------------------------------------------------------
void connectHandBox(){
  disp0[16] = disp1[16] = '\0';
  if (digitalRead(P_HBX_CXN)) return;           // Return if not connected
  if (!timerHandBox) {
    handBoxInUse = 10;                          // Display start-up screen for 10 seconds
    lcdBacklight = true;
    lcdDisplay = true;
    lcdBlink = false;
    lcdCursor = false;
    memcpy(disp0, "  CrayStation   ", 16);
    memcpy(disp1, VERSION, 16);  
  }
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);

  if (handBox.begin(0x20, &Wire)) {
    handBox.pinMode(E_KEY_U,      INPUT_PULLUP);
    handBox.pinMode(E_KEY_D,      INPUT_PULLUP);
    handBox.pinMode(E_KEY_L,      INPUT_PULLUP);
    handBox.pinMode(E_KEY_R,      INPUT_PULLUP);
    handBox.pinMode(E_KEY_ENTER,  INPUT_PULLUP);
    handBox.pinMode(E_KEY_BACK,   INPUT_PULLUP);
    handBox.pinMode(E_BUZZER,     OUTPUT);
  }
  else {
    memcpy(disp0, "*PCF2875 failed*", 16);
    memcpy(disp1, "*Keypad error  *", 16);
  }

  // Power up the RPi if it is not already on
  if (battLevel > LEVEL_BLACK) {
    digitalWrite(P_RPI_PWR, HIGH);
    poweringDown = false;
    setRPiTimers();
  }
  else handboxChargingStatus();

  // Set-up display controls as required
  printLCDbuffer();
  lcdBacklight  ? lcd.backlight() : lcd.noBacklight();
  lcdDisplay    ? lcd.display()   : lcd.noDisplay();
  lcdBlink      ? lcd.blink()     : lcd.noBlink();
  lcdCursor     ? lcd.cursor()    : lcd.noCursor();
  lcd.setCursor(x, y);

  // Inform user that connection has been acknowledged
  beep(100);
  delay(100);
  beep(100);
}
//-------------------------------------------------------------------------------------------------
void handboxChargingStatus(){
  if (checkHandBox()) {
    float v = measureVolts(BATT_V, SCALE_BATT, 20);
    int percent = int((v - 11.37) / 0.013556);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    lcd.noBacklight();
    lcdBacklight = false;
    memcpy(disp0, "BATTERY CRITICAL", 16);
    sprintf(disp1, "Batt: %02.1fV %02d%% ", double(v), percent);
    printLCDbuffer();
  }
}
//-------------------------------------------------------------------------------------------------
void printLCDbuffer() {
  lcd.home();
  lcd.print(disp0);
  lcd.setCursor(0, 1);
  lcd.print(disp1);
  lcd.setCursor(x, y);
}
//-------------------------------------------------------------------------------------------------
void setCursor(){
  if (handBoxInUse) clearLCD();
  if (gettingPwd) {
    c = CANNOT_DO;
    return;
  }

  y = Serial.parseInt();
  x = Serial.parseInt();
  lcd.setCursor(x, y);
}
//-------------------------------------------------------------------------------------------------
void printChar(char chr){
  if (handBoxInUse != 0) clearLCD();                              // 0 means in use

  if (x < LCD_WIDTH &&  y < LCD_HEIGHT) {
    y == 0 ? disp0[x] = chr : disp1[x] = chr;
    lcd.write(chr);
    if (++x >= LCD_WIDTH) {
      x = 0;
      y++;
      lcd.setCursor(x, y);
    } 
  }
}
//-------------------------------------------------------------------------------------------------
void clearLCD(){
  handBoxInUse = 0;                                               // 0 means in use
  for(byte i = 0; i < LCD_WIDTH; i++) disp0[i] = disp1[i] = ' ';
  x = y = 0;
  lcd.clear();
}
//-------------------------------------------------------------------------------------------------
void enterPassword() {
  int pwdPos = 0;
  if (!handBoxHere) {
    gettingPwd = false;
    c = CANNOT_DO;
    return;
  }
  if (gettingPwd) return;             // Password is already being got

  pwdPos = Serial.readBytesUntil('\r', pwd, 63);
  pwd[pwdPos] = '\0';
  gettingPwd = true;
  Serial.println(c);                                                            // Tell RPi that password is being got

  char line[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~ABCDEFGHIJKLMNO";
  byte linePos = 7;
  clearLCD();

  while (true) {
    lcd.noBlink();  lcdBlink = 0;
    lcd.noCursor(); lcdCursor = 0;
    lcd.setCursor(x = 0, y = 0);                                                // Top row shows password chars entered so far
    if (pwdPos) for (byte i = (pwdPos > 15 ? pwdPos - 16 : 0); i < pwdPos; i++) printChar(pwd[i]);
    //if (pwdPos) for (byte i = 0; i <= pwdPos; i++) printChar(char(pwd[i]));
    printChar(' ');

    lcd.setCursor(x = 0, y = 1);
    for (int i = linePos - 7; i <= linePos + 7; i++) printChar(line[i]);        // Bottom row shows cursor and row of chars
    lcd.setCursor(x = 7, y = 1);
    lcd.blink();  lcdBlink = 1;
    lcd.cursor(); lcdCursor = 1;

    lastKey = E_KEY_NONE;
    while (getKey() == 'X') {
      loop();
      if (!checkHandBox() || !gettingPwd) {
        gettingPwd = false;                                                     // Quit if handbox is disconnected or getting password no longer required
        return;
      }
    }

    switch(lastKey) {
      case E_KEY_U:
        if (pwdPos < 63) {
          pwd[pwdPos ++] = line[linePos];
          pwd[pwdPos] = '\0';
        }
        break;
      case E_KEY_D:
        pwdPos = 0;
        pwd[pwdPos == 0] = '\0';
        clearLCD();
        break;
      case E_KEY_R:     
        if (--linePos < 7) linePos += 95;
        break;
      case E_KEY_L:
        if (++linePos > 102)  linePos -= 95;
        break;
      case E_KEY_ENTER:
        c = '\0';
        lcd.noBlink();  lcdBlink = 0;
        lcd.noCursor(); lcdCursor = 0;
        clearLCD();
        gettingPwd = false;
        return;
        break;
      case E_KEY_BACK:
        pwd[pwdPos] = '\0';
        if (pwdPos) pwdPos--;
        break;
    }
    lastKey = E_KEY_NONE;
  }
}
//-------------------------------------------------------------------------------------------------
byte getKey() {
  c = 'X';
  readKeys();
  if (pressedKey) {
    switch (pressedKey){                                  // A key is currently being pressed
      case E_KEY_U:       c = 'u';  break;                // Up     Pin 1
      case E_KEY_D:       c = 'd';  break;                // Down   Pin 2
      case E_KEY_L:       c = 'l';  break;                // Left   Pin 3
      case E_KEY_R:       c = 'r';  break;                // Right  Pin 4
      case E_KEY_ENTER:   c = 'e';  break;                // Enter  Pin 5
      case E_KEY_BACK:    c = 'b';  break;                // Back   Pin 6
    }
    return c;                                             // Return a lowercase letter for pressed key
  }
  switch (lastKey){                                       // Decode last unregistered key press
    case E_KEY_U:       c = 'U';  break;                  // Up     Pin 1
    case E_KEY_D:       c = 'D';  break;                  // Down   Pin 2
    case E_KEY_L:       c = 'L';  break;                  // Left   Pin 3
    case E_KEY_R:       c = 'R';  break;                  // Right  Pin 4
    case E_KEY_ENTER:   c = 'E';  break;                  // Enter  Pin 5
    case E_KEY_BACK:    c = 'B';  break;                  // Back   Pin 6
  }
  if (!gettingPwd) lastKey = E_KEY_NONE;                  // lastKey has now be registered - reset it
  return c;
}
//-------------------------------------------------------------------------------------------------
void readKeys() {
  byte currentKey = E_KEY_NONE;
  for (byte i = 1; i <= 6; i++) if (!handBox.digitalRead(i)) currentKey = i;  // Find a pressed key
  delay(20);                                                                  // Account for debounce
  if (currentKey != E_KEY_NONE) if (handBox.digitalRead(currentKey)) currentKey = E_KEY_NONE;

  if (currentKey == pressedKey) return;                                       // Exit if key still being pressed

  if (currentKey == E_KEY_NONE && pressedKey != E_KEY_NONE) {                 // Has key been released?
    lastKey = pressedKey;                                                     // If so record the key that was pressed
    pressedKey = E_KEY_NONE;                                                  // Key no longer being pressed
    return;
  }
  pressedKey = currentKey;                                                    // Record key currently being pressed
  beep(2);                                                                    // Beep as new key is now being pressed
}
//-------------------------------------------------------------------------------------------------
void getPassword() {
  if (gettingPwd) {
    c = CANNOT_DO;
    return;
  }
  c = '\0';
  Serial.println(pwd);
}
//-------------------------------------------------------------------------------------------------
void soundBuzzer() {
  if (int b = Serial.parseInt() > 0) beep(b);
}
//-------------------------------------------------------------------------------------------------
void beep(int b){
//  handBox.begin(0x20, &Wire);
//  handBox.pinMode(E_BUZZER, OUTPUT);              // Start hanbox buzzer if connected
//  handBox.digitalWrite(E_BUZZER, HIGH);
  digitalWrite(P_BUZZER, HIGH);                   // Start buzzer
  delay(b);                                       // Wait a little
  digitalWrite(P_BUZZER, LOW);                    // Stop buzzer
//  handBox.digitalWrite(E_BUZZER, HIGH);           // Stop hanbox buzzer if connected
}
//=================================================================================================
#pragma endregion
#pragma region MISCELLANEOUS  
//=================================================================================================
// MISCELLANEOUS
//=================================================================================================
void serviceGPS() {
  char b;
  while (GPSserial.available()) {
    b = GPSserial.read();
    if (b == '$') GPSlinePos = 0;
    if (b >= ' ') GPSsentence[GPSline][GPSlinePos++] = b;
    if (b == '\n' || GPSlinePos == 81) {
      GPSsentence[GPSline][GPSlinePos] = '0';
      GPSline = !GPSline;
      GPSlinePos = 0;
    }
  }
}
//-------------------------------------------------------------------------------------------------
void systemStatus(){
  Serial.println(F("\nNANO"));
  for (byte i = 0 ; i <= A7; i++){
    Serial.print(i > 13 ? 'A' : 'D');
    Serial.print(i > 13 ? i - 14 : i);
    Serial.print('\t');
    Serial.print(digitalRead(i));
    Serial.print(' ');
    Serial.print(analogRead(i));
    Serial.print('\t');
    switch(i) {
      case 0: Serial.print(F(">TXD")); break;
      case 1: Serial.print(F("<RXD")); break;
      case 2: Serial.print(F("<OP_SW")); break;
      case 3: Serial.print(F("<CL_SW")); break;
      case 4: Serial.print(F(">RPI_PWR")); break;
      case 5: Serial.print(F(">MTR_PWM")); break;
      case 6: Serial.print(F(">MTR_DIR")); break;
      case 7: Serial.print(F(">SRV_PWR")); break;
      case 8: Serial.print(F(">TEL_PWR")); break;
      case 9: Serial.print(F("<GPS IN")); break;
      case 10: Serial.print(F(">CAM_HTR")); break;
      case 11: Serial.print(F(">SRV_SIG")); break;
      case 12: Serial.print(F("<RAIN_SIG")); break;
      case 13: Serial.print(F(">LED_PWR")); break;
      case A0: Serial.print(F(">GPS PWR")); break;
      case A1: Serial.print(F(">BUZZER")); break;
      case A2: Serial.print(F("<AMPS")); break;
      case A3: Serial.print(F("<HBX_CXN")); break;
      case A4: Serial.print(F("-SDA")); break;
      case A5: Serial.print(F(">SCL")); break;
      case A6: Serial.print(F("<BAT_V")); break;
      case A7: Serial.print(F(">SOL_V")); break;
    }
    Serial.println(); 
  }

 Serial.println(F("\nMODULES"));
  if (bme.begin(0x76)) {
    Serial.print(F("Temp\t"));  printTemperature();
    Serial.print(F("Humid\t")); printHumidity();
    Serial.print(F("Barom\t")); printPressure();
  } else (Serial.println(F("No BME280")));

  if (RTC.begin()) {
    Serial.print(F("RTC Temp\t"));      Serial.print(RTC.temperature() / 100.0, 1);
    Serial.print(F("C\nRTC Time\t"));   getRTCtime();
  } else (Serial.println(F("No DM3231M")));

  Serial.print(F("Last GPS\t"));
  Serial.println(GPSsentence[GPSline]);

  Serial.println(F("\nSETTINGS"));
  Serial.print(F("Srv home angle\t"));  Serial.println(ee.servoHome);
  Serial.print(F("Srv push angle\t"));  Serial.println(ee.servoPush);
  Serial.print(F("Pulse period\t"));    Serial.println(ee.pulsePeriod);
  Serial.print(F("Lid timeout\t"));     Serial.println(ee.lidTimeOut);
  Serial.print(F("Secs past hour\t"));  Serial.println(secsPastHour);
  Serial.print(F("Secs stay awake\t")); Serial.println(ee.secsStayAwake);

  Serial.println(F("\nHANDBOX"));
  if (checkHandBox()) {
    Serial.println(disp0);
    Serial.println(disp1);
    Serial.print(F("\nDisp\t"));    Serial.println(lcdDisplay);
    Serial.print(F("Light\t"));     Serial.println(lcdBacklight);
    Serial.print(F("Cursor\t"));    Serial.println(lcdCursor);
    Serial.print(F("Blink\t"));     Serial.println(lcdBlink);
    Serial.print(F("x\t"));         Serial.println(x);
    Serial.print(F("y\t"));         Serial.println(y);
    Serial.println();

    if (handBox.begin(0x20, &Wire)) {
      for (byte i = 0; i <= 17; i++) {
        Serial.print(F("IO"));
        Serial.print(i);
        Serial.print('\t');
        Serial.print(handBox.digitalRead(i));
        Serial.print('\t');
        switch(i) {
          case 1: Serial.print(F("<KEY_B")); break;
          case 2: Serial.print(F("<KEY_E")); break;
          case 3: Serial.print(F("<KEY_D")); break;
          case 4: Serial.print(F("<KEY_U")); break;
          case 5: Serial.print(F("<KEY_L")); break;
          case 6: Serial.print(F("<KEY_R")); break;
          case 10: Serial.print(F(">BUZZER")); break;
          default: Serial.print('-');
        }
        Serial.println();
        if (i == 7) i+= 2;
      }
    } else Serial.println(F("No PCF2875"));
  } else Serial.println(F("No HandBox"));

  Serial.println("\nEEPROM");
  char buf[17];
  char a;
  for (int i = 0; i < 1024; i+= 16){
    sprintf(buf, "%03X:", i);
    Serial.print(buf);
    for (byte j = 0; j < 16; j++) {
      sprintf(buf, " %02X", EEPROM.read(i+j));
      Serial.print(buf);
    }
    Serial.print(' ');
    for (byte j = 0; j < 16; j++) {
      a = EEPROM.read(i + j);
      if (a >= 32 && a <= 127) Serial.print(a);
      else Serial.print('.');
    }
    Serial.println();
  }

  Serial.println(F("END"));
  c='\0';
}
//-------------------------------------------------------------------------------------------------
void restart() {
  EEPROM.put(1014, clock);                      // Save Arduino clock's settings so that it can be resurrected later
  asm volatile("jmp 0");
}
//-------------------------------------------------------------------------------------------------
void updateEEPROM() {
  int addr;
  struct eeprom xx;

  EEPROM.get(0, addr);
  if(addr > 1022 || addr == 0) addr = 2;

  EEPROM.get(addr, xx);
  while (addr < 990) {
    // Check to see if any settings have changed
    if( (ee.reset != xx.reset) || (ee.servoHome != xx.servoHome) || 
        (ee.servoPush != xx.servoPush) || (ee.lidTimeOut != xx.lidTimeOut)) {
      EEPROM.put(addr, ee);                             // Update EEPROM setting
      EEPROM.get(addr, xx);                             // Read it in to verify was written successfully
      EEPROM.put(addr, ee);
      addr++;                                           // Move to next address if couldn't write to last
    } 
    else break;
  }
}
//-------------------------------------------------------------------------------------------------
void getVersion(){
  Serial.println(VERSION);
  c = '\0';
}
//=================================================================================================
#pragma endregion
#pragma region NOTES
//=================================================================================================
// NOTES
//=================================================================================================
/* Arduino Nano Connections
   ========================
                        +---------+
                D1 TXD  |  ICSP   | VIN
                D0 RXD  |  ooo1   | GND
                RESET   |  ooo    | RESET
                GND     |         | 5V
OP_SW           D2 INT0 |         | A7        BAT_V
CL_SW           D3~INT1 |         | A6        SOL_V
RPI_PWR         D4      |         | A5  SCL   LCD/EXP BOARD/BME280     
MTR_PWM         D5~     |         | A4  SDA   LCD/EXP BOARD/BME280
MTR_DIR         D6~     |         | A3        HBX_CXN
SRV_PWR         D7      |         | A2        AMPS
TEL_PWR         D8      |         | A1        BUZZER
SPARE           D9~     |         | A0        SPARE (GPS POWER)
CAM_HTR         D11~    |  +---+  | 3V3
SRV_SIG         D10~    |         | AREF
RAIN            D12     |  |USB|  | D13       LED
                        +--|   |--+
                           +---+
                ICSP:
                ooo1  RST D13 D12
                ooo    5V D11 GND 

SPARE D9 receives GPS module's data output.

  CrayStation Control Box Connections
  ===================================
  GX12 pin connectors (panel mounted) configurations

  Looking into the socket with pins pointing towards viewer, pin numbers increase anti-clockwise 
  starting from the key notch.

  HBX Handbox
      1 CXN (Be)  Plug connects this pin to ground to indicate handbox is connected
      2 SCL (Y)
      3 SDA (W)
      4 5V  (R)
      5 GND (Bk)

  MAN Manual controller
    A DPDT 3 postion latching switch in a small handheld box, when connected bypasses electronics
    to allow the opening/closing of the observatory lid
      1 12V-in
      2 GND
      3 M1
      4 M2
-
  TEL Telescope connector, provides power to scope for charging, power to servo and control
      1 USB 5V    Connected to +5V on a USB C male plug
      2 USB GND   Connected to GND on a USB C male plug
      3 SRV 5V    Connected to +5V on servo
      4 SRV SIG   Connected to signal input on servo
      5 SRV GND   Connected to GND on servo
      6 n.c.

  LED Power to unltra bright LEDs so that webcam can see inside enclosure
      1 LED 5V
      2 GND
      3 n.c.

  ENC Environment sensor
      1 CXN
      2 SCL
      3 SDA
      4 5V
      5 GND

  LID Wires to devices connected to the enclosure lid
      1 RPi 5V
      2 GND
      3 RAIN 5V
      4 RAIN SIG
      5 GND
      6 HTR 12V

  PWR Power lines in
      1 +12V
      2 GND
      3 SOLAR +VE

  MTR Motor power lines and limit switch lines
      1 OPN
      2 CLS
      3 M1
      4 M2
*/
#pragma endregion
