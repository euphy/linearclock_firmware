/*
Linear Clock driver
Copyright Sandy Noble (sandy.noble@gmail.com) 2019
This version 23rd March 2019

Uses DS3231 Real time clock module wired:
ESP32 pin 21 to DS3231 SDA pin.
ESP32 pin 22 to DS3231 SCL pin.

Uses 2x ULN2003 amps drivers wired to arduino pins 15, 2, 0 & 1 driving minute hand motor,
and pins 34, 35, 32 & 33 driving hour hand motor.  
This is an extremely simple circuit intended to use the most commonly available 
cheap motors (28BYJ-48) and drivers.

Home switches are on pins D2 and D3, they are the AVR interrupts 0 and 1.  They are
electrically wired to switches at both ends of each rail.  

@TODO  Reserve pins for LDR and lighting controller (PWM).
@TODO  Write clock size into EEPROM when it changes so it doesn't have to recalibrate every time it resets.
@TODO  Reserve pins for physical buttons.
@TODO  

*/

// Includes...
// For motor control
#include <AccelStepper.h>

// For i2s realtime clock
#include <Wire.h> 
#include "RTClib.h"

// For wifi web server to set time
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>


// Clock setup. Doesn't mention pins because uses default 21/22 on ESP32.
RTC_DS3231 rtc;

// These set up the motors.  The values here depend on how they've been wired up.
#define MOTOR_A_ENABLE_PIN 27
#define MOTOR_A_STEP_PIN 14
#define MOTOR_A_DIR_PIN 12

#define MOTOR_B_ENABLE_PIN 13
#define MOTOR_B_STEP_PIN 4
#define MOTOR_B_DIR_PIN 15

AccelStepper minuteHand(AccelStepper::DRIVER, MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN);
AccelStepper hourHand(AccelStepper::DRIVER, MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN);

float maxSpeed = 8000.0;
float acceleration = 8000.0;
byte stepSize = 8;

boolean motorsEnabled = false;

/* This is the number of steps to move the indicator from one end to the other. */
static int stepsPerClockMinuteHand = 2105;
static int stepsPerClockHourHand = 2105;

float stepsPerMinute = stepsPerClockMinuteHand/60.0;
float stepsPerHourMinute = stepsPerClockHourHand/(60.0 * 12.0);
float stepsPerHour = stepsPerClockHourHand/12.0;

int const END_MARGIN = 4*stepSize;

// Assume hands are in middle of machine on startup.
// Minutes setup
int startMinutePos = stepsPerClockMinuteHand / 2;
int currentMinutePos = startMinutePos;

// Hours setup
int startHourPos = stepsPerClockHourHand / 2;
int currentHourPos = startHourPos;

// These are the actual time, in seconds minutes and hours.  
// findTimeToDisplay() writes these values, and renderTime() reads them.
int currentSeconds = 0;
int currentMinutes = 0;
int currentHours = 0;

int checkedTimeCount = 0;

/* User interface */
long lastDebugMessageTime = 0L;

/* Limits and interrupts */
boolean machineHasLimitSwitches = true;
boolean machineHasHardLimits = false;

volatile boolean minuteStartLimitTriggered = false;
volatile boolean minuteEndLimitTriggered = false;
volatile boolean hourStartLimitTriggered = false;
volatile boolean hourEndLimitTriggered = false;

int minuteStartLimitPin = 39;
int minuteEndLimitPin = 16;
int hourStartLimitPin = 35;
int hourEndLimitPin = 17;

enum direction {BACKWARD, FORWARD};

direction mDir = BACKWARD;
direction hDir = BACKWARD;

static boolean minuteWinding = true;
static boolean hourWinding = true;

boolean debugToSerial = false;

/* HTTP server setup */
boolean httpServerInitialised = false;
const char *ssid = "linearclock";
AsyncWebServer server(80);


void IRAM_ATTR minuteStartLimitISR()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50) {
    minuteStartLimitTriggered = true;
  }
  lastInterrupt = interruptTime;
}

void IRAM_ATTR hourStartLimitISR()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50) {
    hourStartLimitTriggered = true;
  }
  lastInterrupt = interruptTime;
}

void IRAM_ATTR minuteEndLimitISR()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50) {
    minuteEndLimitTriggered = true;
  }
  lastInterrupt = interruptTime;
}

void IRAM_ATTR hourEndLimitISR()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50) {
    hourEndLimitTriggered = true;
  }
  lastInterrupt = interruptTime;
}

void setup() 
{
  Serial.begin(115200);
  Serial.println("LINEAR CLOCK.");

  if (machineHasLimitSwitches) {
    // attach limit interrupts
    pinMode(minuteStartLimitPin, INPUT_PULLUP);
    attachInterrupt(minuteStartLimitPin, minuteStartLimitISR, FALLING);
  
    pinMode(hourStartLimitPin, INPUT_PULLUP);
    attachInterrupt(hourStartLimitPin, hourStartLimitISR, FALLING);

    pinMode(minuteEndLimitPin, INPUT_PULLUP);
    attachInterrupt(minuteEndLimitPin, minuteEndLimitISR, FALLING);
  
    pinMode(hourEndLimitPin, INPUT_PULLUP);
    attachInterrupt(hourEndLimitPin, hourEndLimitISR, FALLING);
  }
  else {
    Serial.println("No limit switches on this machine.");
  }
  
  minuteHand.setEnablePin(MOTOR_A_ENABLE_PIN);
  minuteHand.setPinsInverted(false, false, true);

  hourHand.setEnablePin(MOTOR_B_ENABLE_PIN);
  hourHand.setPinsInverted(false, false, true);
  
  // setup RTC stuff
  Wire.begin();
  boolean rtcBegun = rtc.begin();

  if (rtcBegun) {
    Serial.println("RTC is started");
  } else {
    Serial.println("RTC couldn't be found!");
    while (1); 
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the default time.");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  } else {
    Serial.println("Picked up time from RTC: ");
    findTimeToDisplay();
    reportPosition();
  }

  // Set up motors and steps
  minuteHand.setMaxSpeed(maxSpeed);
  minuteHand.setAcceleration(acceleration);
  hourHand.setMaxSpeed(maxSpeed);
  hourHand.setAcceleration(acceleration);
  
  recalculateStepsPerUnits();
  if (machineHasHardLimits) {
    detectMachineStartPosition();
    detectMachineEndPosition();
  }
  else {
    Serial.println("Machine has no hard limits, so setting start position to defaults");
    minuteHand.setCurrentPosition(startMinutePos);
    hourHand.setCurrentPosition(startHourPos);
    reportPosition();    
  }

  http_initServer();
}

void recalculateStepsPerUnits() {
  stepsPerMinute = stepsPerClockMinuteHand/60.0;
  stepsPerHourMinute = stepsPerClockHourHand/(60.0 * 12.0);
  stepsPerHour = stepsPerClockHourHand/12.0;
}

void loop() 
{
  findTimeToDisplay();
  setHandPositions();
  moveHands();
  dealWithLimits();

  debug();
}


void debug()
{
  long now = millis();
  if ((now - lastDebugMessageTime) > 2000)
  {
    reportPosition();
    lastDebugMessageTime = now;
  }
}




void moveHands()
{
  if (hourHand.distanceToGo() >= stepSize || minuteHand.distanceToGo() >= stepSize)
  {
    if (!motorsEnabled) {
      hourHand.enableOutputs();
      minuteHand.enableOutputs();
      motorsEnabled = true;
    }

    if (debugToSerial) {
      Serial.print("Moving to ");
      Serial.print(currentHourPos);
      Serial.print(":");
      Serial.print(currentMinutePos);
      Serial.print(" (");
      Serial.print(hourHand.distanceToGo());
      Serial.print(":");
      Serial.print(minuteHand.distanceToGo());
      Serial.println(")");
    }
    

    if (hourEndLimitTriggered)
      Serial.println("hour end limit triggered.");
    else {
      if (debugToSerial) Serial.println("running hour hand!");
      hourHand.run();
    }
      
    if (minuteEndLimitTriggered)
      Serial.println("minute end limit triggered.");
    else {
      if (debugToSerial) Serial.println("running minute hand!");
      minuteHand.run();
    }
    
  }
  else
  {
    hourHand.disableOutputs();
    minuteHand.disableOutputs();
    motorsEnabled = false;
    if (debugToSerial) {
      Serial.println("After moving: ");
      reportPosition();
    }
  }
}


void reportPosition()
{
  Serial.print("Position: ");
  Serial.print(hourHand.currentPosition());
  Serial.print(":");
  Serial.print(minuteHand.currentPosition());
  Serial.print(" (");
  Serial.print(currentHours);
  Serial.print(":");
  Serial.print(currentMinutes);
  Serial.print(":");
  Serial.print(currentSeconds);
  Serial.println(")");
}  

void findTimeToDisplay()
{
  checkedTimeCount++;
  DateTime now = rtc.now();
  if (debugToSerial) {
    serialPrintTime(now);
  }

  int normalisedHours = (now.hour() >= 12) ? now.hour() - 12 : now.hour();
  if (normalisedHours != currentHours || now.minute() != currentMinutes) {
    Serial.print("findTimeToDisplay ran ");
    Serial.print(checkedTimeCount);
    Serial.println(" times before needing to change current time.");
    checkedTimeCount = 0;
  }


  currentHours = now.hour();
  if (currentHours >= 12)
    currentHours = currentHours - 12;
  currentMinutes = now.minute();
  currentSeconds = now.second();

  
  
}

void serialPrintTime(DateTime t) {
  Serial.print(t.year(), DEC);
  Serial.print('/');
  Serial.print(t.month(), DEC);
  Serial.print('/');
  Serial.print(t.day(), DEC);
  Serial.print(' ');
  Serial.print(t.hour(), DEC);
  Serial.print(':');
  Serial.print(t.minute(), DEC);
  Serial.print(':');
  Serial.print(t.second(), DEC);
  Serial.println();
}

void setHandPositions()
{
  displayHour(currentHours, currentMinutes);
  displayMinute(currentMinutes);
}

void displayHour(long hour, long minute)
{
  // work out the new position and set it globally eg time 4:25.
  // first hours
  // eg 0.2055 * 4 * 60 = 49.333
  float justHours = stepsPerHourMinute * hour * 60;
  // eg 0.2055 * 25 = 5.13888
  float justMinutes = stepsPerHourMinute * minute;
  
  // stick em together: position is 54.472 (4:25)
  long oldHourPos = currentHourPos;
  currentHourPos = justHours + justMinutes;

  if (currentHourPos != oldHourPos)
  {
    Serial.print("Moving hour hand to ");
    Serial.print(currentHourPos);
    Serial.print(" (hr pos: ");
    Serial.print(justHours);
    Serial.print(" + min pos: ");
    Serial.print(justMinutes);
    Serial.print("), to represent ");
    Serial.print(hour);
    Serial.print(" hours and ");
    Serial.print(minute);
    Serial.println(" minutes.");
    hourHand.moveTo(currentHourPos*stepSize);
  }
}

void displayMinute(long minute)
{
  long oldMinutePos = currentMinutePos;
  // work out the new position and set it globally
  // eg 2.467 * 25 = 61.675
  currentMinutePos = stepsPerMinute * minute;

  if (currentMinutePos != oldMinutePos)
  {
    Serial.print("Moving minute hand to position ");
    Serial.print(currentMinutePos);
    Serial.print(", to represent ");
    Serial.print(minute);
    Serial.println(" minutes.");
    
    minuteHand.moveTo(currentMinutePos*stepSize);
  }
}

void panic(String message)
{
  Serial.print("PANIC! ");
  Serial.println(message);
  while (1);
}
