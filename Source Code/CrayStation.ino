#define VERSION "CrayStation V0.241215"     
// Started code on 13 Oct 2024.  For Arduino Nano.
// Make sure monitorPulse is set to 'true' in code below for live operation
#pragma region GLOBAL SETTINGS
#include <Servo.h>
#include <Adafruit_PCF8575.h>               // For expansion board        0x20
#include <Adafruit_BME280.h>                // For environment board      0x76
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>              // For handbox LCD display    0x27
#include <Wire.h>

/* Arduino Nano Connections
   ========================
                        +---------+
                D1 TXD  |  ICSP   | VIN
                D0 RXD  |  ooo1   | GND
                RESET   |  ooo    | RESET
                GND     |         | 5V
BOX_OPEN_SW     D2 INT0 |         | A7        SOLAR
BOX_CLOSED_SW   D3~INT1 |         | A6        BATTERY
MTR_DIR         D4      |         | A5  SCL        
MTR_PWM         D5~     |         | A4  SDA
SKYCAM_HEATER   D6~     |         | A3        SERVO
RPI_POWER       D7      |         | A2        AMMETER
SCOPE_POWER     D8      |         | A1        RAIN_RG9
WEBCAM_POWER    D9~     |         | A0        HANDBOX
LED_POWER       D10~    |         | AREF
**SPARE**       D11~    |  +---+  | 3V3
SENSOR_POWER    D12     |  |USB|  | D13       BUZZER
                        +--|   |--+
                           +---+
                ICSP:
                ooo1  RST D13 D12
                ooo    5V D11 GND 
*/
#define BOX_OPEN_SW   2
#define BOX_CLOSED_SW 3
#define MTR_DIR       4
#define MTR_PWM       5
#define MTR_BUTTON    6           // External button to open/stop/close enclosure
#define SKYCAM_HEATER 6           // Dew heater power for the all-sky camera  
#define RPI_POWER     7           // Directly controls power to RPi
#define SCOPE_POWER   8           // Power for charging scope
#define WEBCAM_POWER  9           // Power for the Webcam
#define LED_POWER     10          // Power for the enclosure LEDs
#define SENSOR_POWER  12          // Powers Temp, Humidity, Air Pressure, RG9, Ammeter, Servo
#define BUZZER        13
#define HANDBOX       A0
#define RAIN_RG9      A1
#define AMMETER       A2
#define SERVO         A3
#define BATTERY       A6
#define SOLAR         A7

#define KEY_NONE      0
#define KEY_U         1           // Keypad buttons
#define KEY_D         2
#define KEY_L         3
#define KEY_R         4
#define KEY_ENTER     5           // Enter
#define KEY_BACK      6           // Back
#define EXP_BUZZER    8           // Handbox buzzer

#define SCALE_AMPS    100         // 5A module = 185, 20A = 100, 30A = 66
#define SCALE_BATT    1
#define SCALE_SOLAR   1

#define LCD_WIDTH     16
#define LCD_HEIGHT    2

Servo myServo;
Adafruit_PCF8575 i2cBoard;
Adafruit_BME280 bme;                      // Thermometer, barometer, hygrometer (humidity)
LiquidCrystal_I2C lcd(0x27,  16, 2);      // I2C address 0x27, 16 characters per row, 2 rows

unsigned long timerPulse;
unsigned long pulsePeriod = 5000;
unsigned long timerRamp;
unsigned long rampSpeed = 2;
unsigned long timerBox;
unsigned long boxTimeOut = 30000;

char c;
volatile byte pwm = 0;                    // Motor stopped
byte dir = 1;                             // Motor direction for closing.

byte x = 0;
byte y = 0;
bool handBoxHere = false;
byte lastKey = KEY_NONE;
byte pressedKey = KEY_NONE;
bool lcdBacklight, lcdDisplay, lcdBlink, lcdCursor;
char disp0[17];
char disp1[17];
char pwd[64];                   // Maximum length of a WiFi password is 63 chars
byte udgs[8][8];
bool gettingPwd = false;

bool monitorPulse = false;      // Must be set to true when this code is live
bool actOnRain = true;
bool actOnBattery = true;

#pragma endregion
#pragma region SETUP & LOOP
//=================================================================================================
// SETUP & LOOP
//=================================================================================================
void setup() {
  pinMode(MTR_DIR,        OUTPUT);                  // Define digital pins
  pinMode(MTR_PWM,        OUTPUT);
  pinMode(BUZZER,         OUTPUT);
  pinMode(RPI_POWER,      OUTPUT);
  pinMode(SCOPE_POWER,    OUTPUT);
  pinMode(WEBCAM_POWER,   OUTPUT);
  pinMode(LED_POWER,      OUTPUT);
  pinMode(SENSOR_POWER,   OUTPUT);
  pinMode(RAIN_RG9,       INPUT_PULLUP);
  pinMode(BOX_OPEN_SW,    INPUT_PULLUP);
  pinMode(BOX_CLOSED_SW,  INPUT_PULLUP);
  pinMode(HANDBOX,        INPUT_PULLUP);
  pinMode(SOLAR,          INPUT);
  pinMode(BATTERY,        INPUT);
  pinMode(AMMETER,        INPUT);
  pinMode(11, OUTPUT);

  digitalWrite(11, 1);
  analogWrite(MTR_PWM,          LOW);               // No power to the motor
  digitalWrite(MTR_DIR,         LOW);               // Default to 'Open'
  digitalWrite(BUZZER,          LOW);
  digitalWrite(RPI_POWER,       HIGH);
  digitalWrite(WEBCAM_POWER,    LOW);
  digitalWrite(SCOPE_POWER,     LOW);
  
  while(!Serial);
  Serial.begin(9600);
  Serial.setTimeout(50);
  myServo.attach(SERVO);
  if (!bme.begin(0x76)) Serial.println("BME280 sensor not responding!");
  if (!i2cBoard.begin(0x20, &Wire)) Serial.println("PCF8575 I2C expansion board not responding!");

  Serial.println("Ready");

  timerPulse = timerRamp = millis();                // Initialise timers
  dir = digitalRead(BOX_CLOSED_SW);                 // Make last motor direction equal the status of closed switch (0 = open, 1 = close)

  lcdBacklight = true;                              // Initialise handbox's LCD parameters
  lcdDisplay = true;
  lcdBlink = false;
  lcdCursor = false;
  for (byte i = 0; i < 16; i++) disp0[i] = disp1[i] = ' ';
  disp0[16] = disp1[16] = '\0';
  pwd[0] = '\0';
  memcpy(disp0, "CrayStation", 11);
  connectHandBox();
  attachInterrupt(digitalPinToInterrupt(BOX_OPEN_SW), stopMotor, LOW);
  attachInterrupt(digitalPinToInterrupt(BOX_CLOSED_SW), stopMotor, LOW);
  closeBox();
} 
//=================================================================================================
void loop() {
  batteryCheck();                       // Check battery charge - close box if it is too low
  checkRain();                          // Check if raining - close box if it is
  serviceMotor();                       // Control motor and monitor enclosure switches
  commands();                           // Service Comms from RPi
  checkPulse();
  checkHandBox();
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
  timerPulse = millis() + pulsePeriod;          // Comms was received - RPi is still working

  switch (c){
    case 'O': openBox();                break;
    case 'o': openSwitchStatus();       break;
    case 'C': closeBox();               break;
    case 'c': closedSwitchStatus();     break;
    case 'm': motorStatus();            break;
    case 'X': stopMotor();              break;

    case 'S': telescopeSwitch();        break;  // Make servo press telescope's on/off button
    case 'B': baseSwitch();             break;
    case 'W': webcamPower();            break;  // Turn on/off power to the webcam
    case 'L': leds();                   break;  // Turn on/off power to the LEDs
    case 'D': dewHeater();              break;  // Turn on/off power to the skycam's dew heater

    case '#': checkHandBox();           break;  // Sees if the handbox is connected
    case 'k': getKey();                 break;  // Returns pressed key info on handbox
    case 'e': getExpBoardPins();        break;  // Look at raw input from Expansion board in handbox
    case '{': printText();              break;  // Hand box display
    case '@': setCursor();              break;
    case '$': escapeCodes();            break;
    case '=': serialPrintRows();        break;  // Print what is currently shown on the handbox display
    case 'N': enterPassword();          break;  // Handbox handles input of a WiFi password
    case 'n': getPassword();            break;  // Returns the password set
    case 'g': c = '0' + gettingPwd;     break;  // See if a password is being got by the handbox

    case '~': setPulsePeriod();         break;  // How long to wait for comms from the RPi
    case 'P': monitor_Pulse();          break;  // Watch for comms from RPi
    case 'p': ignore_Pulse();           break;  // Don't watch for comms from RPi

    case 'r': checkRain();              break;
    case 'R': setRainAction();          break;  // Toggles whether to close lid if raining
    case 't': getTemperature();         break;
    case 'h': getHumidity();            break;
    case 'b': getBarometer();           break;
    case 'v': getBatteryVolts();        break;
    case 'U': setBatteryAction();       break;  // Toggles whether to close lid if battery charge is low
    case 'u': getSolarVolts();          break;
    case 'a': getAmps();                break;  // 12V battery amps currently being consumed
    case 'Z': soundBuzzer();            break;  // Sound buzzer (& handbox buzzer) for a given period
   
    case 'I': getVersion();             break;
    case '*': asm volatile ("jmp 0");   break;  // Restart the Arduino
    case '?': help();                   break;  // Summary of commands available to the RPi or user
    case '\n':  c = '\0';               break;
    case '\r':  c = '\0';               break;
    default:    c = '?';                break;  // Unknown command
  }
  if (c != '\0') {
    Serial.println(c);
  }
}
//=================================================================================================
#pragma endregion
#pragma region SWITCHES
//=================================================================================================
void telescopeSwitch(){
  int pressTime = 0;                        // Get how long to press the telesope power button for in ms.
  pressTime = Serial.parseInt();
  if (batteryGood()) {
    if (pressTime > 1){
      myServo.write(131);                   // Press the power button
      delay(pressTime);
      myServo.write(90);                    // Return servo arm
    }
  }
  else c ='#';                              // Unable to carry out operation
}
//-------------------------------------------------------------------------------------------------
void baseSwitch(){
  int pressTime = 0;                        // Get how long to press the telesope reset button for in ms.
  pressTime = Serial.parseInt();
  if (batteryGood()) {
    if (pressTime > 1){
      myServo.write(0);                     // Pull the cord!
      delay(pressTime);
      myServo.write(90);                    // Return servo arm
    }
  }
  else c = '#';                             // Unable to carry out operation
}
//-------------------------------------------------------------------------------------------------
void webcamPower(){
  if (batteryGood()) {
    switch (Serial.parseInt()) {
      case '0': digitalWrite(WEBCAM_POWER, LOW);    // Turn off the enclosure webcam - L9110S Dual-Channel H-bridge
      case '1': digitalWrite(WEBCAM_POWER, HIGH);   // Turn on the enclosuer webcam  - L9110S Dual-Channel H-bridge
    }
  }
  else c = '#';                                     // Unable to carry out operation
}
//-------------------------------------------------------------------------------------------------
void leds(){
  if (batteryGood()) {
    switch (Serial.parseInt()) {
      case '0': digitalWrite(LED_POWER, LOW);       // Turn off the enclosure webcam - L9110S Dual-Channel H-bridge
      case '1': digitalWrite(LED_POWER, HIGH);      // Turn on the enclosuer webcam  - L9110S Dual-Channel H-bridge
    }
  }
  else c = '#';                                     // Unable to carry out operation
}
//-------------------------------------------------------------------------------------------------
void dewHeater(){
  int value = 0;
  value = Serial.parseInt();
  if (batteryGood()) {
    if (value > 255) analogWrite(SKYCAM_HEATER, value);
  }
  else c = '#';
}
//=================================================================================================
#pragma endregion
#pragma region ENVIRONMNENT
//=================================================================================================
// ENVIRONMENT
//=================================================================================================
void batteryCheck() {
  if (!batteryGood()) {                         // Battery is at critical level.  Turn-off everyhing to save power
    digitalWrite(WEBCAM_POWER, LOW);
    digitalWrite(LED_POWER, LOW);
    digitalWrite(SCOPE_POWER, LOW);
    digitalWrite(RPI_POWER, LOW);
    digitalWrite(BUZZER, LOW);
    digitalWrite(SENSOR_POWER, LOW);
    if (actOnBattery) closeBox();               // Close the box if it is open.

    unsigned long beepTimer = millis();
    while (!batteryGood()) {                    // Wait until the battery has enough charge and beep every 10secs
      serviceMotor();                                             // Finish closing box if it is closing
      if (millis() < beepTimer + 100) digitalWrite(BUZZER, 1);    // Start buzzer - allow it to sound for 100ms
      if (millis() < beepTimer + 100) digitalWrite(BUZZER, 0);    // Turn off buzzer after 100ms
      if (millis() < beepTimer + 10000) beepTimer = millis();     // Reset beepTimer after 10secs
    }
  }
  digitalWrite(RPI_POWER, HIGH);                // Provide power to start/restart RPi
}
//-------------------------------------------------------------------------------------------------
byte batteryGood() {
  float v = measureVolts(BATTERY, SCALE_BATT, 20);
  if (v < 12.1) return 0;                       // Battery needs charging - turn off all systems;
  if (v < 12.6) return 1;                       // Systems can operate but don't do any observations
  return 2;                                     // Go for it!
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
void getBatteryVolts(){
  c = '\0';
  printFloat(measureVolts(SOLAR, SCALE_BATT, 20), 1);       // Needs to be in the form like 13.7V
  Serial.println('V');
}
//-------------------------------------------------------------------------------------------------
void getSolarVolts(){
  c = '\0';
  printFloat(measureVolts(SOLAR, SCALE_SOLAR, 20), 1);      // Needs to be in the form like 13.7V
  Serial.println('V');
}
//-------------------------------------------------------------------------------------------------
void getAmps(){
  // https://www.amazon.co.uk/s?k=ACS712+20A&crid=3SKQ3JVK7WWHD&sprefix=acs712+20a%2Caps%2C63&ref=nb_sb_noss
  // AZDelivery ACS712 20A
  // https://cdn.shopify.com/s/files/1/1509/1638/files/AZ043_A_8-6_EN_B0736DYV3W_c2659f11-4c11-4aaf-8f7e-3d84ee0c3b9a.pdf?v=1721208630
  // SCALE_AMPS = 5A module = 185, 20A = 100, 30A = 66

  float average = 0;                                  // Take an average of 100 readings to get a stable reading
  for (byte i = 0; i < 100; i++) {
    average += analogRead(AMMETER);
    delay(1);
  }
  average /= 100;
  float voltage = (average / 1023.0) * 5000.0;
  float current = (voltage - 2500.0) / SCALE_AMPS;    // 2500 may need to be changed after calibration for 0 amps

  c = '\0';
  Serial.print(int(current * 1000.0));                // Return milli-amps
  Serial.println("mA");
}
//-------------------------------------------------------------------------------------------------
void getTemperature(){
  // https://cdn.shopify.com/s/files/1/1509/1638/files/AZ109_A3-7_EN_B07D8T4HP6_068a8db7-3572-4ef5-b396-be75b7ec8810.pdf?v=1721047611
  printFloat(bme.readTemperature() - 2, 1);
  Serial.println('C');
  c = '\0';
}
//-------------------------------------------------------------------------------------------------
void getHumidity(){
  // https://cdn.shopify.com/s/files/1/1509/1638/files/AZ109_A3-7_EN_B07D8T4HP6_068a8db7-3572-4ef5-b396-be75b7ec8810.pdf?v=1721047611
  printFloat(bme.readHumidity(),  1);
  Serial.println('%');
  c = '\0';
}
//-------------------------------------------------------------------------------------------------
void getBarometer(){
  // https://cdn.shopify.com/s/files/1/1509/1638/files/AZ109_A3-7_EN_B07D8T4HP6_068a8db7-3572-4ef5-b396-be75b7ec8810.pdf?v=1721047611
  printFloat(bme.readPressure() / 100.0F, 1);
  Serial.println("mb");
  c = '\0';
}
//-------------------------------------------------------------------------------------------------
void checkRain(){
  // Use for Hydreon RG-9 rain sensor.  Intructions:
  // https://rainsensors.com/wp-content/uploads/sites/3/2022/03/2022.02.17-rev-1.200-rg-9_instructions.pdf

  c = '0' + !digitalRead(RAIN_RG9);                 // '0' = dry, '1' = raining
  if (c == '1' && actOnRain) closeBox();            //  If enclosure is open then close enclosure
}
//-------------------------------------------------------------------------------------------------
void setRainAction() {                // Toggles whether to close lid if raining or not
  actOnRain = 1 - actOnRain;
  c = '0' +  actOnRain;               // '0' means lid won't close if raining, '1' means it will
}
//-------------------------------------------------------------------------------------------------
void setBatteryAction() {             // Toggles whether to close lid if battery charge is too low or not
  actOnBattery = 1 - actOnBattery;
  c = '0' +  actOnBattery;            // '0' means lid won't close if low battery charge, '1' means it will
}
//-------------------------------------------------------------------------------------------------
void printFloat(float value, byte decimalPlaces){
  if (value < 0) {                                          // Print sign of number if -ve
    Serial.print('-');
    value = -value;                                         // Make number positive
  }

  value = value + 5 / pow(10, decimalPlaces) + 0.00000001;  // Rounding by adding 5/(10^x)  [x = decimalPlaces]
  Serial.print(int(value));                                 // Print the integer part first
  if (decimalPlaces == 0) return;                           // Return if no decimal places to print

  Serial.print('.');                                        // Decimal point
  String numStr = String(value - int(value));               // Print the decimal places
  for (byte i = 0; i < decimalPlaces; i++) Serial.print(numStr[i]);
}

//=================================================================================================
#pragma endregion
#pragma region PULSE
//=================================================================================================
void setPulsePeriod(){
  int temp;
  temp = Serial.parseInt();
  if (temp > 0) pulsePeriod = temp;
}
//-------------------------------------------------------------------------------------------------
void checkPulse(){
  if (!monitorPulse) return;
  if (float(analogRead(BATTERY)) * SCALE_BATT > 12.2) {
    if (millis() > timerPulse) {                      // RPi pulse not detected within pulse period?
      gettingPwd = false;                             // Password will no longer being got, if it was being got
      digitalWrite(RPI_POWER, 0);                     // Reset RPi by cutting power
      delay(10000);                                   // Wait 10 seconds
      digitalWrite(RPI_POWER, 1);                     // Power up the RPi
      timerPulse = millis() + 20000;                  // Wait a while for RPi to re-awaken (20secs)
    }
  }
}
//-------------------------------------------------------------------------------------------------
void monitor_Pulse() {
  monitorPulse = true;
}
//-------------------------------------------------------------------------------------------------
void ignore_Pulse() {
  monitorPulse = false;
}
//=================================================================================================
#pragma endregion
#pragma region ENCLOSURE OPENING CONTROL
//=================================================================================================
//  ENCLOSURE CONTROL
//=================================================================================================
void openSwitchStatus(){
  c = '0' + !digitalRead(BOX_OPEN_SW);                  // '0' = Open, '1' = Closed
  if (millis() >= timerBox) {
    if (c == '0') {                                     // If switch is sill open...
      stopMotor();                                      // something has gone wrong - STOP!!!
      c = '2';                                          // '2' = Still open and timed out
    }
  }
}
//-------------------------------------------------------------------------------------------------
void closedSwitchStatus(){
  c = '0' + !digitalRead(BOX_CLOSED_SW);                // '0' = Open, '1' = Closed
  if (millis() >= timerBox) {
    if (c == '0') {                                     // If switch is sill open...
      stopMotor();                                      // something has gone wrong - STOP!!!
      c = '2';                                          // '2' = Still open and timed out
    }
  }
}
//-------------------------------------------------------------------------------------------------
void openBox(){
  if(digitalRead(BOX_CLOSED_SW)) {                    // Only start motor if not open
    if (dir == 1) stopMotor();                        // Stop motor if lid is closing
    pwm = 1;                                          // Start ramping up motor
    dir = 0;                                          // Open box
    timerBox = millis() + boxTimeOut;                 // Setup timeout
  }
}
//-------------------------------------------------------------------------------------------------
void closeBox(){
  if(digitalRead(BOX_OPEN_SW)) {                      // Only start motor if not closed
    if (dir == 0) stopMotor();                        // Stop if motor is closing
    pwm = 1;                                          // Start ramping up motor
    dir = 1;                                          // Close box
    timerBox = millis() + boxTimeOut;                 // Setup timeout
  }
}
//-------------------------------------------------------------------------------------------------
void serviceMotor(){
  serviceMotorButton();                               // See if the external box button has been pressed
  if (millis() >= timerBox && pwm > 0) stopMotor();   // Something has gone wrong - STOP!!!

  if (dir) {                                          // dir = 0 open enclosure, dir = 1 close enclosure.
    if (!digitalRead(BOX_CLOSED_SW)) stopMotor();     // Stop motor if closed
  } 
  else {
    if (!digitalRead(BOX_OPEN_SW)) stopMotor();       // Stop motor if open
  }

  if (pwm > 0) {                                      // If pwm > 0 then ramp up motor
    if (pwm == 1) {                                   // Intitialise stuff when starting to ramp up (pwm will be 1)
      timerRamp = millis();
      if (dir) {
        detachInterrupt(digitalPinToInterrupt(BOX_OPEN_SW));                    // Reset interrupt for open switch
        attachInterrupt(digitalPinToInterrupt(BOX_CLOSED_SW), stopMotor, LOW);  // Set up interrupt for closed switch
      }
      else {
        detachInterrupt(digitalPinToInterrupt(BOX_CLOSED_SW));                  // Reset interrupt for closed switch
        attachInterrupt(digitalPinToInterrupt(BOX_OPEN_SW), stopMotor, LOW);    // Set up interrupt for open switch
      }
    }

    digitalWrite(MTR_DIR, dir);                       // Motor direction (0 = open, 1 = close)
    digitalWrite(MTR_PWM, pwm);                       // Motor speed

    if(millis() >= timerRamp){
      if (pwm < 255) pwm++;                           // Ramp up to a maximum value of 255
      else timerRamp+= rampSpeed;                     // Use this one for a consistent ramp-up time, but some steps may be missed - probably won't matter

      //else timerRamp = millis() + rampSpeed;        // Use this for a smooth ramp-up
    }
  } else stopMotor();
}
//-------------------------------------------------------------------------------------------------
void stopMotor(){                                         // This is also used as an Interrupt Service Routine

// Either the open or closed switched, or the motor button has been pressed
  detachInterrupt(digitalPinToInterrupt(BOX_OPEN_SW));    // Reset interrupt for closed switch
  detachInterrupt(digitalPinToInterrupt(BOX_CLOSED_SW));  // Reset interrupt for open switch
  digitalWrite(MTR_PWM, pwm = 0);                         // Stop the motor, pwm = 0 means motor has stopped                 
}
//-------------------------------------------------------------------------------------------------
void serviceMotorButton(){
  if (digitalRead(BOX_OPEN_SW) || digitalRead(BOX_CLOSED_SW)) return; // Both switches will both appear closed if button pressed.  Return if not.
  while (!digitalRead(BOX_OPEN_SW) && !digitalRead(BOX_CLOSED_SW)) {  // Wait for the button to be released
      if (pwm > 0) return stopMotor();                                // If motor is moving then stop it and then quit
      beep(100);
  }
  dir = 1 - dir;                                                      // Move motor in opposite to last direction
  pwm = 1;                                                            // Start motor
}
//-------------------------------------------------------------------------------------------------
void motorStatus(){
  c = '0' + (pwm > 0);                                // '0' = Stopped, '1' = Moving
  if (millis() >= timerBox) {
    if (c == '0') {                                   // If switch is sill open...
      stopMotor();                                    // something has gone wrong - STOP!!!
      c = '2';                                        // '2' = Still open and timed out
    }
  }
}
//=================================================================================================
#pragma endregion
#pragma region HANDBOX
//=================================================================================================
// HANDBOX
//=================================================================================================
bool checkHandBox() {
  return handBoxHere = true;                        // ***REMOVE WHEN GOING LIVE***
  if (!digitalRead(HANDBOX)) {                      // Handbox attached?
    if (handBoxHere) {
      handBoxHere = false;                          // Handbox was connected but not now
      c = '0';
    } else {
      handBoxHere = true;                           // Handbox has just been connected
      c = '1';
      connectHandBox();
    }
  } else handBoxHere = false;
  return handBoxHere;
}
//-------------------------------------------------------------------------------------------------
void printText(){
  if (gettingPwd) {
    c = '%';
    return;
  }

  unsigned long timer = millis() + 1000;
  char chr[2];
  while (timer > millis()){
  Serial.readBytes(chr, 1);
    switch (chr[0]) {
      case '\0':  return;                   break;
      case '\r':  return;                   break;
      case '\n':  return;                   break;
      case '}':   return;                   break;
      case '@':   setCursor();              break;
      case '$':   escapeCodes();            break;
      default:    printChar(chr[0]);        break;
    }
  }
}
//-------------------------------------------------------------------------------------------------
void setCursor(){
  if (gettingPwd) {
    c = '%';
    return;
  }

  y = Serial.parseInt();
  x = Serial.parseInt();
  lcd.setCursor(x, y);
}
//-------------------------------------------------------------------------------------------------
void escapeCodes(){
  if (gettingPwd) {
    c = '%';
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
    case '{': printChar('{');                       break;  // Print characters used as escape codes
    case '}': printChar('}');                       break;
    case '@': printChar('@');                       break;
    case '$': printChar('$');                       break;
    case 'U': createChar();                         break;  // Create a user defined graphic (UDG)
    case '0': printChar(byte(0));                   break;  // Print UDGs
    case '1': printChar(byte(1));                   break;
    case '2': printChar(byte(2));                   break;
    case '3': printChar(byte(3));                   break;
    case '4': printChar(byte(4));                   break;
    case '5': printChar(byte(5));                   break;
    case '6': printChar(byte(6));                   break;
    case '7': printChar(byte(7));                   break;
    case 'H': lcd.setCursor(x = 0, y = 0);          break;  // Cursor home
    case 'N': lcd.setCursor(x = 0, ++y);            break;  // Move cursor down a row
    case 'R': lcd.setCursor(x = 0, y);              break;  // Return cursor to start of row
    case 'X': clearLCD();                           break;  // Clear screen & return cursor home
  }
}
//-------------------------------------------------------------------------------------------------
void printChar(char chr){
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
  for(byte i = 0; i < LCD_WIDTH; i++) disp0[i] = disp1[i] = ' ';
  x = y = 0;
  lcd.clear();
}
//-------------------------------------------------------------------------------------------------
void createChar(){                                  // $X,b1,b2,b3,b4,b5,b6,b7$
  //byte udg[8];                                      // Eight bytes make a User Defined Graphic
  byte charCode = byte(Serial.parseInt());          // Get byte to use for UDG (0 to 7)
  for (byte i = 0; i <= 7; i++){                    // Get 8x 6 bit bytes which form the UDG
    udgs[charCode][i] = Serial.parseInt();
  }
  lcd.createChar(charCode, udgs[charCode]);
}
//-------------------------------------------------------------------------------------------------
void serialPrintRows(){
  c = '\0';
  for(byte i = 0; i < LCD_WIDTH; i++) {
    if (disp0[i] >= 32) Serial.print(disp0[i]);
    else Serial.write(disp0[i] + '0');
  }
  Serial.println('<');
  for(byte i = 0; i < LCD_WIDTH; i++) {
    if (disp1[i] >= 32) Serial.print(disp1[i]);
    else Serial.write(disp0[i] + '0');
  }
  Serial.println('<');
}
//-------------------------------------------------------------------------------------------------
void connectHandBox(){
  i2cBoard.begin(0x20, &Wire);
  i2cBoard.pinMode(KEY_U,      INPUT_PULLUP);
  i2cBoard.pinMode(KEY_D,      INPUT_PULLUP);
  i2cBoard.pinMode(KEY_L,      INPUT_PULLUP);
  i2cBoard.pinMode(KEY_R,      INPUT_PULLUP);
  i2cBoard.pinMode(KEY_ENTER,  INPUT_PULLUP);
  i2cBoard.pinMode(KEY_BACK,   INPUT_PULLUP);
  i2cBoard.pinMode(EXP_BUZZER, OUTPUT);
  
  lcd.init();
  lcd.clear();
  for (byte i = 0; i <-7; i++) lcd.createChar(i, udgs[i]);
  for (byte i = 0; i <= 15; i++) lcd.write(disp0[i]);
  lcd.setCursor(0, 1);
  for (byte i = 0; i <= 15; i++) lcd.write(disp1[i]);
  lcdBacklight  ? lcd.backlight() : lcd.noBacklight();
  lcdDisplay    ? lcd.display()   : lcd.noDisplay();
  lcdBlink      ? lcd.blink()     : lcd.noBlink();
  lcdCursor     ? lcd.cursor()    : lcd.noCursor();
  lcd.setCursor(x, y);
}
//-------------------------------------------------------------------------------------------------
void enterPassword() {
  if (gettingPwd) {
    c = '%';
    return;
  }
  Serial.println(c);                                                            // Tell RPi that password is being got

  int pwdPos = Serial.readBytesUntil('\r', pwd, 63);
  pwd[pwdPos] = '\0';
  gettingPwd = true;

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

    while (getKey() == 'X') {
      loop();
      if (!checkHandBox() || !gettingPwd) {
        gettingPwd = false;                                                     // Quit if handbox is disconnected or getting password no longer required
        return;
      }
    }

    switch(lastKey) {
      case KEY_U:
        pwd[pwdPos ++] = line[linePos];
        pwd[pwdPos] = '\0';
        break;
      case KEY_D:
        pwd[pwdPos] = '\0';
        if (pwdPos > 0) pwdPos--;
        break;
      case KEY_L:     
        if (--linePos < 7) linePos += 95;
        break;
      case KEY_R:
        if (++linePos > 102)  linePos -= 95;
        break;
      case KEY_ENTER:
        gettingPwd = false;
        
        return;
        break;
      case KEY_BACK:
        pwdPos = 0;
        pwd[0] = '\0';
        break;
    }
    lastKey = KEY_NONE;
  }
}
//-------------------------------------------------------------------------------------------------
void getExpBoardPins() {
  Serial.print("Pins input values: ");
  for (byte i = 0; i <= 15; i++) Serial.println(i2cBoard.digitalRead(i));
  Serial.println();
  c = '\0';
}//-------------------------------------------------------------------------------------------------
byte getKey() {
  c = 'X';
  readKeys();
  if (pressedKey) {
    switch (pressedKey){                                  // A key is currently being pressed
      case KEY_U:       c = 'u';  break;                  // Up     Pin 1
      case KEY_D:       c = 'd';  break;                  // Down   Pin 2
      case KEY_L:       c = 'l';  break;                  // Left   Pin 3
      case KEY_R:       c = 'r';  break;                  // Right  Pin 4
      case KEY_ENTER:   c = 'e';  break;                  // Enter  Pin 5
      case KEY_BACK:    c = 'b';  break;                  // Back   Pin 6
    }
    return c;                                             // Return a lowercase letter for pressed key
  }
  switch (lastKey){                                       // Decode last unregistered key press
    case KEY_U:       c = 'U';  break;                    // Up     Pin 1
    case KEY_D:       c = 'D';  break;                    // Down   Pin 2
    case KEY_L:       c = 'L';  break;                    // Left   Pin 3
    case KEY_R:       c = 'R';  break;                    // Right  Pin 4
    case KEY_ENTER:   c = 'E';  break;                    // Enter  Pin 5
    case KEY_BACK:    c = 'B';  break;                    // Back   Pin 6
  }
  if (!gettingPwd) lastKey = KEY_NONE;                    // lastKey has now be registered - reset it
  return c;
}
//-------------------------------------------------------------------------------------------------
void readKeys() {
  byte currentKey = KEY_NONE;
  for (byte i = 1; i <= 6; i++) if (!i2cBoard.digitalRead(i)) currentKey = i; // Find a pressed key
  delay(20);                                                                  // Account for debounce
  if (currentKey != KEY_NONE) if (i2cBoard.digitalRead(currentKey)) currentKey = KEY_NONE;

  if (currentKey == pressedKey) return;                                       // Exit if key still being pressed

  if (currentKey == KEY_NONE && pressedKey != KEY_NONE) {                     // Has key been released?
    lastKey = pressedKey;                                                     // If so record the key that was pressed
    pressedKey = KEY_NONE;                                                    // Key no longer being pressed
    return;
  }
  pressedKey = currentKey;                                                    // Record key currently being pressed
  beep(20);                                                                   // Beep as new key is now being pressed
}
//-------------------------------------------------------------------------------------------------
void getPassword() {
  if (gettingPwd) {
    c = '%';
    return;
  }
  c = '\0';
  Serial.println(pwd);
}
//-------------------------------------------------------------------------------------------------
void soundBuzzer() {
  int b = Serial.parseInt();
  if (b > 0) beep(b);
}
//-------------------------------------------------------------------------------------------------
void beep(int b){
  digitalWrite(BUZZER, 1);                          // Start buzzer
  if (handBoxHere) {
    i2cBoard.pinMode(EXP_BUZZER, OUTPUT);           // Start hanbox buzzer if connected
    i2cBoard.digitalWrite(EXP_BUZZER, 1);
  }
  delay(b);                                         // Wait a little
  digitalWrite(BUZZER, 0);                          // Stop buzzer
  if (handBoxHere) digitalWrite(EXP_BUZZER,1);      // Stop hanbox buzzer if connected
}
//=================================================================================================
#pragma endregion
#pragma region INFORMATION
//=================================================================================================
// INFORMATION
//=================================================================================================
void getVersion(){
  Serial.println(VERSION);
  c = '\0';
}
//-------------------------------------------------------------------------------------------------
void help(){
  Serial.println("Sorry - no help available.  You're on your own - good luck!");
  c = '\0';
}
//=================================================================================================
#pragma endregion
