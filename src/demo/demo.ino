
#include <Wire.h>
#include <Zumo32U4.h>
#include "AssassinatorUtility.h"

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
AssassinatorUtility assUtils;

unsigned int lineSensorValues[3];

// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring.  This value might need to be tuned for
// different lighting conditions, surfaces, etc.
const uint16_t lineSensorThreshold = 1000;

// The speed that the robot uses when backing up.
const uint16_t reverseSpeed = 200;

// The speed that the robot uses when turning.
const uint16_t turnSpeed = 200;

// The speed that the robot usually uses when moving forward.
// You don't want this to be too fast because then the robot
// might fail to stop when it detects the white border.
const uint16_t forwardSpeed = 200;

// These two variables specify the speeds to apply to the motors
// when veering left or veering right.  While the robot is
// driving forward, it uses its proximity sensors to scan for
// objects ahead of it and tries to veer towards them.
const uint16_t veerSpeedLow = 0;
const uint16_t veerSpeedHigh = 250;

// The speed that the robot drives when it detects an opponent in
// front of it, either with the proximity sensors or by noticing
// that it is caught in a stalemate (driving forward for several
// seconds without reaching a border).  400 is full speed.
const uint16_t rammingSpeed = 700;

// The amount of time to spend backing up after detecting a
// border, in milliseconds.
const uint16_t reverseTime = 200;

// The minimum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMin = 200;

// The maximum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMax = 2100;

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
const uint16_t waitTime = 3000;

// If the robot has been driving forward for this amount of time,
// in milliseconds, without reaching a border, the robot decides
// that it must be pushing on another robot and this is a
// stalemate, so it increases its motor speed.
const uint16_t stalemateTime = 4000;

const uint16_t changeModeWaitTime = 5000;

// This enum lists the top-level states that the robot can be in.
enum State
{
  StatePausing,
  StateWaiting,
  StateScanning,
  StateDriving,
  StateBacking,
  StateDefensiveMode,
  StateOffensiveMode,
  StateFighting,
  StateExpandSheild
};

State prevState = StatePausing;
State state = StatePausing;

enum Direction
{
  DirectionLeft,
  DirectionRight,
};

// scanDir is the direction the robot should turn the next time
// it scans for an opponent.
Direction scanDir = DirectionLeft;

// The time, in milliseconds, that we entered the current top-level state.
uint16_t stateStartTime;

// The time, in milliseconds, that the LCD was last updated.
uint16_t displayTime;

// This gets set to true whenever we change to a new state.
// A state can read and write this variable this in order to
// perform actions just once at the beginning of the state.
bool justChangedState;

// This gets set whenever we clear the display.
bool displayCleared;

// Rotating count.
uint16_t rotateCount;

const char march[] PROGMEM = 
"! O2 T100 MS"
"a8. r16 a8. r16 a8. r16 f8 r16 >c16" 
"ML"
"a8. r16 f8 r16" "MS" ">c16 a. r8"
"O3"
"e8. r16 e8. r16 e8. r16 f8 r16 <c16"
"O2 ML"
"a-8. r16" "MS" "f8 r16 >c16 a r"

"O3 ML"
"a8. r16 <a8 r16 <a16" "MS" "a8. r16 a-8 r16 g16"
"ML V10"
"g-16 f16 g-16 r16 r8 <b-16 r16" "MS" "e-8. r16 d8." "ML" "d-16"
"c16 <c-16 c16 r16 r8" "MS O2" "f16 r16 a-8. r16 f8. a-16"
"O3"
"c8. r16 <a8 r16 c16 e2. r8"

"O3 ML"
"a8. r16 <a8 r16 <a16" "MS" "a8. r16 a-8 r16 g16"
"ML V10"
"g-16 f16 g-16 r16 r8 <b-16 r16" "MS" "e-8. r16 d8." "ML" "d-16"
"c16 <c-16 c16 r16 r8" "MS O2" "f16 r16 a-8. r16 f8. >c16"
"ML"
"a8. r16 f8 r16 >c16 a2. r8"
;

void setup() {
  // put your setup code here, to run once:
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  changeState(StatePausing);

  rotateCount = 0;
}

void loop() {
  bool buttonAPressed = buttonA.getSingleDebouncedPress();
  bool buttonBPressed = buttonB.getSingleDebouncedPress();
  bool buttonCPressed = buttonC.getSingleDebouncedPress();

  if (state == StatePausing)
  {
    // In this state, we just wait for the user to press button
    // A, while displaying the battery voltage every 100 ms.

    motors.setSpeeds(0, 0);

    if (justChangedState)
    {
      justChangedState = false;
      lcd.print(F("Press A"));
    }

    if (displayIsStale(100))
    {
      displayUpdated();
      lcd.gotoXY(0, 1);
      lcd.print(readBatteryMillivolts());
    }

    if (buttonAPressed)
    {
      changeState(StateScanning);
    }
    if (buttonBPressed)
    {
      changeState(StateWaiting);
    }
    if (buttonCPressed)
    {
      changeState(StateExpandSheild);
    }
  }
  else if (buttonAPressed)
  {
    // The user pressed button A while the robot was running, so pause.
    changeState(StatePausing);
  }
  else if (state == StateWaiting) {
    uint16_t time = timeInThisState();

    if (time < waitTime)
    {
      // Display the remaining time we have to wait.
      uint16_t timeLeft = waitTime - time;
      lcd.gotoXY(0, 0);
      lcd.print(timeLeft / 1000 % 10);
      lcd.print('.');
      lcd.print(timeLeft / 100 % 10);
    }
    else
    {
      // We have waited long enough.  Start moving.
      changeState(StateExpandSheild);
    }
  }
  else if (state == StateFighting) {
    // Random choose between StateDefensiveMode or StateOffensiveMode
//    if (random(0, 2) == 0) {
//        changeState(StateDefensiveMode);
//    } else {
//        changeState(StateOffensiveMode);
//    }
    changeState(StateOffensiveMode);
  }
  else if (state == StateExpandSheild) {
    if (timeInThisState() < 200) {
      motors.setSpeeds(400, 400); 
    } else if (timeInThisState() >= 200 && timeInThisState() < 400) {
      motors.setSpeeds(-400, -400);
    } else {
      changeState(StateFighting);
    }
  }
  else if (state == StateBacking)
  {
    // In this state, the robot drives in reverse.

    motors.setSpeeds(-reverseSpeed, -reverseSpeed);

    // After backing up for a specific amount of time, start
    // scanning.
    if (timeInThisState() >= reverseTime) {
      changeState(StateFighting);
    }
  }
  else if (state == StateScanning) {
    scanMode();
    if (buzzer.isPlaying())
    {  
    }
    else
    {
      buzzer.playFromProgramSpace(march);
    }
  }
  else if (state == StateDefensiveMode)
  {
    if ( timeInThisState() < changeModeWaitTime) {
      defensiveMode();
    }
    else {
      changeState(StateFighting);
    }
  }
  else if (state == StateOffensiveMode)
  {
    if ( timeInThisState() < changeModeWaitTime) {
      offensiveMode();
    }
    else {
      changeState(StateFighting);
    }
  }
}

void offensiveMode()
{
  // Check for borders.
  lineSensors.read(lineSensorValues);

  if (lineSensorValues[0] < lineSensorThreshold)
  {
    scanDir = DirectionRight;
    changeState(StateBacking);
  }
  if (lineSensorValues[2] < lineSensorThreshold)
  {
    scanDir = DirectionLeft;
    changeState(StateBacking);
  }

  // Read the proximity sensors to see if know where the
  // opponent is.
  proxSensors.read();
  uint8_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
  int8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();

  if (sum >= 4 || timeInThisState() > stalemateTime)
  {
    // The front sensor is getting a strong signal, or we have
    // been driving forward for a while now without seeing the
    // border.  Either way, there is probably a robot in front
    // of us and we should switch to ramming speed to try to
    // push the robot out of the ring.
    motors.setSpeeds(rammingSpeed, rammingSpeed);

    // Turn on the red LED when ramming.
    ledRed(1);
  }
  else if (sum == 0)
  {
    // We don't see anything with the front sensor, so just
    // keep driving forward.  Also monitor the side sensors; if
    // they see an object then we want to go to the scanning
    // state and turn torwards that object.

    motors.setSpeeds(forwardSpeed, forwardSpeed);

//    if (proxSensors.countsLeftWithLeftLeds() >= 2)
//    {
//      // Detected something to the left.
//      scanDir = DirectionLeft;
//      changeState(StateScanning);
//    }
//
//    if (proxSensors.countsRightWithRightLeds() >= 2)
//    {
//      // Detected something to the right.
//      scanDir = DirectionRight;
//      changeState(StateScanning);
//    }

    ledRed(0);
  }
  else
  {
    // We see something with the front sensor but it is not a
    // strong reading.

    if (diff >= 1)
    {
      // The right-side reading is stronger, so veer to the right.
      motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
    }
    else if (diff <= -1)
    {
      // The left-side reading is stronger, so veer to the left.
      motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
    }
    else
    {
      // Both readings are equal, so just drive forward.
      motors.setSpeeds(forwardSpeed, forwardSpeed);
    }
    ledRed(0);
  }
}

void defensiveMode()
{
  // Read the proximity sensors to see if know where the opponent is.
  proxSensors.read();

  // Attack if the opponent is close enough.
  if (assUtils.getOpponentDistance(proxSensors) >= 2) {

    // Todo : Check the border before moving forward

    // Move forward to opponent
    motors.setSpeeds(400, 400);

    // Go to rotating mode after attacking for some time.
    if (rotateCount >= 100) {
      rotateCount = 0;
    }
  }
  else
  {
    // Keep rotating.
    assUtils.setRotate(motors, 200);
  }

  rotateCount++;
}

void scanMode()
{
  motors.setSpeeds(0, 0);
  // Read the proximity sensors to see if know where the opponent is.
  proxSensors.read();

  lcd.clear();
  lcd.print("D ");
  lcd.print(assUtils.getOpponentDistance(proxSensors));
  lcd.gotoXY(0, 1);

  lcd.print("L/R ");
  lcd.print(assUtils.getOpponentDirection(proxSensors));
}

// Changes to a new state.  It also clears the LCD and turns off
// the LEDs so that the things the previous state were doing do
// not affect the feedback the user sees in the new state.
void changeState(uint8_t newState)
{
  prevState = state;
  state = (State)newState;
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  lcd.clear();
  displayCleared = true;
}

// Returns true if the display has been cleared or the contents
// on it have not been updated in a while.  The time limit used
// to decide if the contents are staled is specified in
// milliseconds by the staleTime parameter.
bool displayIsStale(uint16_t staleTime)
{
  return displayCleared || (millis() - displayTime) > staleTime;
}

// Any part of the code that uses displayIsStale to decide when
// to update the LCD should call this function when it updates the
// LCD.
void displayUpdated()
{
  displayTime = millis();
  displayCleared = false;
}

uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}
