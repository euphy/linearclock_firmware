void detectMachineStartPosition()
{
  // check to see if any sensors are already triggered and move the carriage if it is
//  clearEndStops(minuteStartLimitPin, minuteHand);
//  minuteStartLimitTriggered = false;
//  clearEndStops(hourLimitPin, hourHand);
//  hLimitTriggered = false;
  
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
    
    if (minuteStartLimitTriggered) {
      minuteWinding = false;
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      // creep until trigger cleared
      minuteHand.move(100*stepSize);
      while (minuteHand.distanceToGo() != 0) {
        Serial.println("minute creeping...");
        Serial.println(minuteHand.distanceToGo());
        minuteHand.run();
        if (digitalRead(minuteStartLimitPin) != LOW) {
          Serial.println("Limit switch released (a).");
          break;
        }
      }
      minuteHand.move(END_MARGIN*stepSize);
      while (minuteHand.distanceToGo() != 0) minuteHand.run();
      
      minuteStartLimitTriggered = false;
      minuteHand.disableOutputs();
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      Serial.println("Minute rewind finished.");
    }
    else if (minuteWinding) {
      minuteHand.move(-stepSize);
      minuteHand.setSpeed(-maxSpeed);
      minuteHand.runSpeed();
      if (debugToSerial) {
        Serial.print("rewind minutes pos ");
        Serial.println(minuteHand.currentPosition());
      }
    }
    
    if (hourStartLimitTriggered) {
      hourWinding = false;
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(hourHand.currentPosition());
      // creep until trigger cleared
      hourHand.move(100*stepSize);
      while (hourHand.distanceToGo() != 0) {
        Serial.println("hour creeping...");
        hourHand.run();
        if (digitalRead(hourStartLimitPin) != LOW) {
          Serial.println("Limit switch released (b).");
          break;
        }
      }
      hourHand.move(END_MARGIN*stepSize);
      while (hourHand.distanceToGo() != 0) hourHand.run();

      hourHand.disableOutputs();
      hourStartLimitTriggered = false;
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(0);
      
      Serial.println("Hour rewind finished.");
    }
    else if (hourWinding) {
      hourHand.move(-stepSize);
      hourHand.setSpeed(-maxSpeed);
      hourHand.runSpeed();
      if (debugToSerial) {
        Serial.print("rewind hours pos ");
        Serial.println(hourHand.currentPosition());
      }
    }
  }
}


void detectMachineEndPosition() 
{
  // see if the end stops are currently low
  if (digitalRead(minuteEndLimitPin) != LOW) {
    Serial.println("Minute End limit switch is triggered while the hand is at the beginning of the clock. This makes no sense, I'm out!");
    while (1);
  }
  if (digitalRead(hourEndLimitPin) != LOW) {
    Serial.println("Hour End limit switch is triggered while the hand is at the beginning of the clock. This makes no sense, I'm out!");
    while (1);
  }

  // so now test the full length
  Serial.println("Winding forward to detect length of clock...");
  minuteWinding = true;
  hourWinding = true;
  minuteHand.enableOutputs();
  hourHand.enableOutputs();
 
  while (minuteWinding || hourWinding) {
    if ((minuteEndLimitTriggered || (digitalRead(minuteEndLimitPin) != LOW)) 
        && minuteWinding) {
      Serial.println("Minute End limit triggered.");
      minuteWinding = false;

      // creep until trigger cleared
      minuteHand.move(-(100*stepSize));
      while (minuteHand.distanceToGo() != 0) {
        Serial.println("min reverse creeping...");
        minuteHand.run();
        if (digitalRead(minuteEndLimitPin) != LOW) {
          Serial.println("Limit switch released (c).");
          break;
        }
      }

      minuteHand.move(-(END_MARGIN*stepSize));
      while (minuteHand.distanceToGo() != 0) minuteHand.run();
      stepsPerClockMinuteHand = minuteHand.currentPosition();
      recalculateStepsPerUnits();
      minuteEndLimitTriggered = false;
      Serial.println("Minute size detection finished.");
    }

    if (minuteWinding) {
      minuteHand.move(stepSize);
      minuteHand.setSpeed(maxSpeed);
      minuteHand.runSpeed();
    }
    
    if ((hourEndLimitTriggered || (digitalRead(hourEndLimitPin) != LOW))
        && hourWinding) {
      hourWinding = false;
      
      // creep backwards until trigger cleared
      hourHand.move(-(100*stepSize));
      while (hourHand.distanceToGo() != 0) {
        Serial.println("hour reverse creeping...");
        hourHand.run();
        if (digitalRead(hourEndLimitPin) != LOW) {
          Serial.println("Limit switch released (d).");
          break;
        }
      }

      hourHand.move(-(END_MARGIN*stepSize));
      while (hourHand.distanceToGo() != 0) hourHand.run();
      stepsPerClockHourHand = hourHand.currentPosition();
      recalculateStepsPerUnits();
      hourEndLimitTriggered = false;
      Serial.println("Hour size detection finished.");
    }
    
    if (hourWinding) {
      hourHand.move(stepSize);
      hourHand.setSpeed(maxSpeed);
      hourHand.runSpeed();
    }      
  }


  minuteHand.setMaxSpeed(maxSpeed);
  mDir = FORWARD;
  hourHand.setMaxSpeed(maxSpeed);
  hDir = FORWARD;

  minuteHand.disableOutputs();
  hourHand.disableOutputs();

  Serial.println("End stops found!!!");
  Serial.print("Minute axis is ");
  Serial.print(stepsPerClockMinuteHand);
  Serial.println(" true steps long.");
  Serial.print("Houre axis is ");
  Serial.print(stepsPerClockHourHand);
  Serial.println(" true steps long.");
}


void dealWithLimits()
{
  if (machineHasLimitSwitches) {
    dealWithMinuteLimits();
    dealWithHourLimits();
  }
}

boolean handIsAtPosition(boolean limitTriggered, int limitPin, String hand, String whichEnd) 
{
  boolean atPosition = false;
  if (limitTriggered) {
    Serial.print(hand);
    Serial.print(" ");
    Serial.print(whichEnd);
    if (digitalRead(limitPin) == LOW) {
      Serial.println(" pin gone from high to low (switch closed)");
      atPosition = true;
    }
    else {
      Serial.print(" pin gone from low to high (switch opened)");
    }
    limitTriggered = false;
  }
  return atPosition;
}

void dealWithMinuteLimits() 
{
  boolean atStart = handIsAtPosition(minuteStartLimitTriggered, minuteStartLimitPin, "minute", "start");
  boolean atEnd = handIsAtPosition(minuteEndLimitTriggered, minuteEndLimitPin, "minute", "end");

  if (machineHasHardLimits) {
    if (atStart && atEnd) {
      panic("Both minute limit switches are closed. That's impossible!");
    }
    else if (atStart && !atEnd) {
      // That's ok, it's at the start      
      Serial.println("Hard minute hand start limit switch closed.");
      if (mDir == BACKWARD) {
        // that makes sense too, backwards _should_ trigger the start trigger
        Serial.println("Minute hand was going backwards at the time, so set current position to be early end of the machine (0).");
        minuteHand.setCurrentPosition(0);
        minuteHand.moveTo(0);
        minuteHand.disableOutputs();        
      }
      else if (mDir == FORWARD) {
        // oh-oh, forwards shouldn't trigger the start trigger
        panic("Minute start triggered, but moving forward.");
      }
    }
    else if (!atStart && atEnd) {
      // That's ok, it's at the end
      if (mDir == BACKWARD) {
        // backwards shouldn't trigger the end trigger
        panic("Minute end triggered, but moving backwards.");
      }
      else if (mDir == FORWARD) {
        // forward should trigger the end trigger
        // recalibrate length of machine
        Serial.println("Hard minute hand end limit switch closed.");
        Serial.print("Minute hand was going forwards at the time, so consider this position to be the late end of the machine, and recalibrate the steps-per-clock based on this (");
        Serial.print(minuteHand.currentPosition());
        Serial.println(").");
        stepsPerClockMinuteHand = minuteHand.currentPosition()-END_MARGIN;
        stepsPerClockMinuteHand = stepsPerClockMinuteHand / stepSize;
        recalculateStepsPerUnits();
        minuteHand.moveTo(minuteHand.currentPosition());
        minuteHand.disableOutputs();        
      }      
    }
  }
  
  minuteStartLimitTriggered = false;
  minuteEndLimitTriggered = false;
}

void dealWithHourLimits() {
  boolean atStart = handIsAtPosition(hourStartLimitTriggered, hourStartLimitPin, "hour", "start");
  boolean atEnd = handIsAtPosition(hourEndLimitTriggered, hourEndLimitPin, "hour", "end");
    
  hourStartLimitTriggered = false;
  hourEndLimitTriggered = false;
}
