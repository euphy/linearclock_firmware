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


void dealWithLimits()
{
  if (machineHasLimitSwitches) {
    dealWithMinuteLimit();
    dealWithHourLimit();
  }
}

void dealWithMinuteLimit() {
  if (mLimitTriggered) {
    if (digitalRead(minuteLimitPin)) {
      Serial.println("Minute pin gone from low to high (switch opened)");
      mLimitTriggered = false;
      return;
    }
    else {
      Serial.println("Minute pin gone from high to low (switch closed)");
    }

    if (machineHasHardLimits) {
      Serial.println("Hard minute hand limit switch closed.");
      if (mDir == BACKWARD) {
        Serial.println("Minute hand was going backwards at the time, so set current position to be early end of the machine (0).");
        minuteHand.setCurrentPosition(0);
        minuteHand.moveTo(0);
        minuteHand.disableOutputs();
      }
      else if (mDir == FORWARD) {
        // recalibrate length of machine
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
    
    mLimitTriggered = false;
  }
}

void dealWithHourLimit() {
  if (hLimitTriggered) {
    if (digitalRead(hourLimitPin)) {
      Serial.println("Hour pin gone from low to high (switch opened)");
      hLimitTriggered = false;
      return;
    }
    else {
      Serial.println("Hour pin gone from high to low (switch closed)");
    }

    if (machineHasHardLimits) {
      Serial.println("Hard hour hand limit switch closed.");
      if (hDir == BACKWARD) {
        Serial.println("Hour hand was going backwards at the time, so set current position to be early end of the machine (0).");
        hourHand.setCurrentPosition(0);
        hourHand.moveTo(0);
        hourHand.disableOutputs();
      }
      else if (hDir == FORWARD) {
        // recalibrate length of machine
        Serial.print("Hour hand was going forwards at the time, so consider this position to be the late end of the machine, and recalibrate the steps-per-clock based on this (");
        Serial.print(hourHand.currentPosition());
        Serial.println(").");
        stepsPerClockHourHand = hourHand.currentPosition()-END_MARGIN;
        stepsPerClockHourHand = stepsPerClockHourHand / stepSize;
        recalculateStepsPerUnits();
        hourHand.moveTo(hourHand.currentPosition());
        hourHand.disableOutputs();
      }
    }
    
    hLimitTriggered = false;
  }
}
