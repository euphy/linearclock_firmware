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
#include <Arduino.h>
// For motor control
#include <AccelStepper.h>

// For i2s realtime clock
#include <Wire.h> 
#include "RTClib.h"

// For wifi web server to set time
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// other project files
#include "http_server.h"


// Clock setup. Doesn't mention pins because uses default 21/22 on ESP32.
RTC_DS3231 rtc;

// These set up the motors.  The values here depend on how they've been wired up.
const byte motoraPin1 = 15;
const byte motoraPin2 = 2;
const byte motoraPin3 = 0;
const byte motoraPin4 = 4;

const byte motorbPin1 = 34;
const byte motorbPin2 = 35;
const byte motorbPin3 = 32;
const byte motorbPin4 = 33;

AccelStepper minuteHand(AccelStepper::FULL4WIRE, motoraPin1, motoraPin2, motoraPin3, motoraPin4); // minutes
AccelStepper hourHand(AccelStepper::FULL4WIRE, motorbPin1, motorbPin2, motorbPin3, motorbPin4); // hours

float maxSpeed = 10000.0;
float acceleration = 5000.0;
byte stepSize = 1;

boolean motorsEnabled = false;

/* This is the number of steps to move the indicator from one end to the other. */
static int stepsPerClockMinuteHand = 2105;
static int stepsPerClockHourHand = 2105;

float stepsPerMinute = stepsPerClockMinuteHand/60.0;
float stepsPerHourMinute = stepsPerClockHourHand/(60.0 * 12.0);
float stepsPerHour = stepsPerClockHourHand/12.0;

int const END_MARGIN = 4;

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


/* User interface */
long lastDebugMessageTime = 0L;

/* Limits and interrupts */
boolean useLimitSwitches = false;
volatile boolean mLimitTriggered = false;
volatile boolean hLimitTriggered = false;

int minuteLimitPin = 12;
int hourLimitPin = 14;
int mInt = 12;
int hInt = 14;

const byte BACKWARD = 0;
const byte FORWARD = 1;

static byte mDir = BACKWARD;
static byte hDir = BACKWARD;

static boolean minuteWinding = true;
static boolean hourWinding = true;

boolean debugToSerial = true;


/* HTTP server setup */
boolean httpServerInitialised = false;
const char *ssid = "linearclock";
WebServer server(80);


void mLimitISR()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50)
    mLimitTriggered = true;
   lastInterrupt = interruptTime;
}

void hLimitISR()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50)
    hLimitTriggered = true;
   lastInterrupt = interruptTime;
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("LINEAR CLOCK.");

  // attach limit interrupts
//  attachInterrupt(digitalPinToInterrupt(mInt), mLimitISR, FALLING);
  pinMode(mInt, INPUT_PULLUP);
  digitalWrite(minuteLimitPin, HIGH);

//  attachInterrupt(digitalPinToInterrupt(hInt), hLimitISR, FALLING);
  pinMode(hInt, INPUT_PULLUP);
  digitalWrite(hourLimitPin, HIGH);

  minuteHand.setMaxSpeed(maxSpeed);
  hourHand.setMaxSpeed(maxSpeed);
  
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
  if (useLimitSwitches) {
    homeHands();
  }
  else {
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
//  moveHands();
  if (useLimitSwitches) {
    dealWithLimits();
  }
//  debug();
  http_handleClient();
}

void dealWithLimits()
{
  if (mLimitTriggered) {
    if (mDir == BACKWARD) {
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(0);
      minuteHand.disableOutputs();
    }
    else if (mDir == FORWARD) {
      // recalibrate length of machine
      stepsPerClockMinuteHand = minuteHand.currentPosition()-END_MARGIN;
      stepsPerClockMinuteHand = stepsPerClockMinuteHand / stepSize;
      recalculateStepsPerUnits();
      minuteHand.moveTo(minuteHand.currentPosition());
      minuteHand.disableOutputs();
    }
    mLimitTriggered = false;
  }

  if (hLimitTriggered) {
    if (hDir == BACKWARD) {
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(0);
      hourHand.disableOutputs();
    }
    else if (hDir == FORWARD) {
      // recalibrate length of machine
      stepsPerClockHourHand = hourHand.currentPosition()-END_MARGIN;
      stepsPerClockHourHand = stepsPerClockHourHand / stepSize;
      recalculateStepsPerUnits();
      hourHand.moveTo(hourHand.currentPosition());
      hourHand.disableOutputs();
    }
    hLimitTriggered = false;
  }
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

void clearEndStops(int &limitPin, AccelStepper &hand)
{
  int distanceToBackOff = -(10*stepSize);
  hand.enableOutputs();
  if (digitalRead(limitPin) == LOW) {
    Serial.println("limit pin is low.");
    for (int range=0; range<(200); range++) {
      Serial.print("wiggling ");
      Serial.println(range);
      // try winding backwards a little bit
      Serial.println("Testing in backwards direction.");
      hand.move(-(range*stepSize));
      while (hand.distanceToGo() != 0) {
        Serial.println("Running backwards...");
        hand.run();
        if (digitalRead(limitPin) != LOW) {
          Serial.println("Limit switch released (1).");
          break;
        }
      }
      // check if that worked,
      if (digitalRead(limitPin) == LOW) {
        // looks like backwards didn't work, lets try forwards the same distance
        Serial.println("Testing in forwards direction.");
        hand.move(2*(range*stepSize));
        while (hand.distanceToGo() != 0) {
          Serial.println("Running forwards...");
          hand.run();
          if (digitalRead(limitPin) != LOW) {
            Serial.println("Limit switch released (2).");
            distanceToBackOff = abs(distanceToBackOff);
            break;
          }
        }
      }
      // if either of these worked, then break out this iterating loop
      if (digitalRead(limitPin) != LOW) {
        Serial.println("Limit switch released (3).");
        break;
      }
    }
    // and back off a bit further
    Serial.print("Backing off ");
    Serial.println(distanceToBackOff);
    hand.move(distanceToBackOff);
    while (hand.distanceToGo() != 0) {
      hand.run();
    }
  }
  
  
  hand.disableOutputs();
}

void homeHands()
{
  // check to see if any sensors are already triggered and move the carriage if it is
  clearEndStops(minuteLimitPin, minuteHand);
  mLimitTriggered = false;
  clearEndStops(hourLimitPin, hourHand);
  hLimitTriggered = false;
  
  // run backwards, slowly until sensor is triggered.
  Serial.println("Minute hand not home.");
  mDir = BACKWARD;
  minuteWinding = true;
  minuteHand.enableOutputs();
  
  Serial.println("Hour hand not at home.");
  hDir = BACKWARD;
  hourWinding = true;
  hourHand.enableOutputs();
  
  Serial.println("Winding backwards.");

  while (minuteWinding || hourWinding) {
    if (mLimitTriggered) {
      minuteWinding = false;
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      // creep until trigger cleared
      minuteHand.move(100*stepSize);
      while (minuteHand.distanceToGo() != 0) {
        Serial.println("minute creeping...");
        Serial.println(minuteHand.distanceToGo());
        minuteHand.run();
        if (digitalRead(minuteLimitPin) != LOW) {
          Serial.println("Limit switch released (a).");
          break;
        }
      }
      minuteHand.move(END_MARGIN*stepSize);
      while (minuteHand.distanceToGo() != 0) minuteHand.run();
      
      mLimitTriggered = false;
      minuteHand.disableOutputs();
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      Serial.println("Minute rewind finished.");
    }
    if (minuteWinding) {
      minuteHand.move(-stepSize);
      minuteHand.setSpeed(-maxSpeed);
      minuteHand.runSpeed();
      if (debugToSerial) {
        Serial.print("rewind minutes pos ");
        Serial.println(minuteHand.currentPosition());
      }
    }
    
    if (hLimitTriggered) {
      hourWinding = false;
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(hourHand.currentPosition());
      // creep until trigger cleared
      hourHand.move(100*stepSize);
      while (hourHand.distanceToGo() != 0) {
        Serial.println("hour creeping...");
        hourHand.run();
        if (digitalRead(hourLimitPin) != LOW) {
          Serial.println("Limit switch released (b).");
          break;
        }
      }
      hourHand.move(END_MARGIN*stepSize);
      while (hourHand.distanceToGo() != 0) hourHand.run();

      hourHand.disableOutputs();
      hLimitTriggered = false;
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(0);
      
      Serial.println("Hour rewind finished.");
    }
    if (hourWinding) {
      hourHand.move(-stepSize);
      hourHand.setSpeed(-maxSpeed);
      hourHand.runSpeed();
      if (debugToSerial) {
        Serial.print("rewind hours pos ");
        Serial.println(hourHand.currentPosition());
      }
    }
  }


  // so now test the full length
  Serial.println("Winding forward to detect length of clock...");
  minuteWinding = true;
  hourWinding = true;
  minuteHand.enableOutputs();
  hourHand.enableOutputs();
 
  while (minuteWinding || hourWinding) {
    if (mLimitTriggered && minuteWinding) {
      Serial.println("Minute limit triggered.");
      minuteWinding = false;

      // creep until trigger cleared
      minuteHand.move(-(100*stepSize));
      while (minuteHand.distanceToGo() != 0) {
        Serial.println("min reverse creeping...");
        minuteHand.run();
        if (digitalRead(minuteLimitPin) != LOW) {
          Serial.println("Limit switch released (c).");
          break;
        }
      }

      minuteHand.move(-(END_MARGIN*stepSize));
      while (minuteHand.distanceToGo() != 0) minuteHand.run();
      stepsPerClockMinuteHand = minuteHand.currentPosition();
      recalculateStepsPerUnits();
      mLimitTriggered = false;
      Serial.println("Minute size detection finished.");
    }

    if (minuteWinding) {
      minuteHand.move(stepSize);
      minuteHand.setSpeed(maxSpeed);
      minuteHand.runSpeed();
//      Serial.println("forward wind minutes...");
    }
    if (hLimitTriggered && hourWinding) {
      hourWinding = false;
      
      // creep until trigger cleared
      hourHand.move(-(100*stepSize));
      while (hourHand.distanceToGo() != 0) {
        Serial.println("hour reverse creeping...");
        hourHand.run();
        if (digitalRead(hourLimitPin) != LOW) {
          Serial.println("Limit switch released (d).");
          break;
        }
      }

      hourHand.move(-(END_MARGIN*stepSize));
      while (hourHand.distanceToGo() != 0) hourHand.run();
      stepsPerClockHourHand = hourHand.currentPosition();
      recalculateStepsPerUnits();
      hLimitTriggered = false;
      Serial.println("Hour size detection finished.");
    }
    if (hourWinding) {
      hourHand.move(stepSize);
      hourHand.setSpeed(maxSpeed);
      hourHand.runSpeed();
//      Serial.println("forward wind hours...");
    }      
  }


  minuteHand.setMaxSpeed(maxSpeed);
  mDir = FORWARD;
  hourHand.setMaxSpeed(maxSpeed);
  hDir = FORWARD;

  minuteHand.disableOutputs();
  hourHand.disableOutputs();

  Serial.println("HOMED!!");
  Serial.print("Minute axis is ");
  Serial.print(stepsPerClockMinuteHand);
  Serial.println(" true steps long.");
  Serial.print("Houre axis is ");
  Serial.print(stepsPerClockHourHand);
  Serial.println(" true steps long.");
}

void moveHands()
{
  if (hourHand.distanceToGo() >= stepSize || minuteHand.distanceToGo() >= stepSize)
  {
    if (!motorsEnabled) {
      hourHand.enableOutputs();
      minuteHand.enableOutputs();
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
    

    if (hLimitTriggered)
      Serial.println("hlimit triggered.");
    else {
      if (debugToSerial) Serial.println("running hour hand!");
      hourHand.run();
    }
      
    if (mLimitTriggered)
      Serial.println("mlimit triggered.");
    else {
      if (debugToSerial) Serial.println("running minute hand!");
      minuteHand.run();
    }
    if (hourHand.distanceToGo() < stepSize || minuteHand.distanceToGo() < stepSize) {
      hourHand.disableOutputs();
      minuteHand.disableOutputs();
    }
    
  }
  else
  {
    if (debugToSerial) {
      Serial.println("After moving: ");
      reportPosition();
    }
  }
}


String reportPosition()
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
  DateTime now = rtc.now();
  if (debugToSerial) {
//    serialPrintTime(now);
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
    Serial.print(", to represent ");
    Serial.println(hour);
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
    Serial.println(minute);
    
    minuteHand.moveTo(currentMinutePos*stepSize);
  }
}
