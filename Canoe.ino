#include <Encoder.h>
#include <FlexyStepper.h>
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>

// Define constants
const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 4;
const int DIR_PIN = 6;
const int STEP_PIN = 3;
const int ENABLE_PIN = 5;
const int MODE_SELECT_PIN_1 = 7;
const int MODE_SELECT_PIN_2 = 8;
const int HEADING_HOLD_MODE = 0;
const int SET_MODE = 1;
const int MANUAL_MODE = 2;
int currentCount;
float currentHeading;
float targetHeading;
float targetPosition;
float headingError;
int encoderCount = 0;
unsigned long currentTime = millis();

// Define variables
int mode = HEADING_HOLD_MODE; // Default to heading hold mode
bool modeSelectPin1State = false;
bool modeSelectPin2State = false;

// Initialize objects
FlexyStepper stepper;
HMC5883L_Simple compass;
Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);

// Interrupt service routine for mode select pins
void handleModeSelect() {
  modeSelectPin1State = digitalRead(MODE_SELECT_PIN_1);
  modeSelectPin2State = digitalRead(MODE_SELECT_PIN_2);
}

// Initialize mode select pins
void initializeModeSelect() {
  pinMode(MODE_SELECT_PIN_1, INPUT_PULLUP);
  pinMode(MODE_SELECT_PIN_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_SELECT_PIN_1), handleModeSelect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MODE_SELECT_PIN_2), handleModeSelect, CHANGE);
}

// Initialize encoder
void initializeEncoder() {
  encoder.write(0);
}

// Initialize stepper
void initializeStepper() {
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  stepper.connectToPins(DIR_PIN, STEP_PIN);
  digitalWrite(ENABLE_PIN, HIGH); // Disable stepper initially
  stepper.setSpeedInStepsPerSecond(50000 * 16); // Set speed to 50,000 steps per second
  stepper.setAccelerationInStepsPerSecondPerSecond(2000 * 16); // Set acceleration to 2000 steps per second^2
  stepper.setStepsPerRevolution(1600); // Set steps per revolution to 1600
}

void initializeCompass() {
  compass.SetDeclination(16.0, 0.0, 'W'); // Set declination for Campbell River, BC, Canada
  compass.SetScale(COMPASS_SCALE_810); // Set scale to COMPASS_SCALE_810
}

void setup() {
  initializeModeSelect();
  initializeEncoder();
  initializeStepper();
  initializeCompass();
}


void loop() {
  

  // Update mode based on mode select pins
  if (!modeSelectPin1State && modeSelectPin2State) {
    mode = HEADING_HOLD_MODE;
    digitalWrite(ENABLE_PIN, LOW); // Enable stepper during Heading Hold mode
  } else if (modeSelectPin1State && !modeSelectPin2State) {
    mode = SET_MODE;
    digitalWrite(ENABLE_PIN, HIGH); // Disable stepper during Set mode
  } else if (!modeSelectPin1State && !modeSelectPin2State) {
    mode = MANUAL_MODE;
    digitalWrite(ENABLE_PIN, LOW); // Enable stepper during Manual mode
  }

  // Perform appropriate action based on mode
  switch (mode) {
    case HEADING_HOLD_MODE:
  static float initialHeading = 0.0; // Declare initialHeading as a static variable
  static unsigned long lastCompassReadTime = 0; // Declare lastCompassReadTime as a static variable
  
  // Store initial heading when mode is first entered
  if (initialHeading == 0.0) {
    initialHeading = compass.GetHeadingDegrees();
  }

  // Read current heading from compass every half second
  currentTime = millis();
  if (currentTime - lastCompassReadTime >= 500) {
    lastCompassReadTime = currentTime;
    currentHeading = compass.GetHeadingDegrees();
  }

  // Calculate target heading based on initial heading
  targetHeading = initialHeading;

  // Calculate error between current and target heading
  headingError = currentHeading - targetHeading;

  // Adjust error to be within -180 to 180 range
  if (headingError > 180) {
    headingError -= 360;
  } else if (headingError < -180) {
    headingError += 360;
  }

  // Set target position of stepper motor in rotations based on heading error and direction

  if (headingError > 0) {
    targetPosition = map(headingError, 0, 90, 0, 4);
    targetPosition = map(targetPosition, 0, 4, 0, 8);
  } else {
    targetPosition = map(headingError, -90, 0, -4, 0);
    targetPosition = map(targetPosition, -4, 0, -8, 0);
  }

  // Set target position of stepper motor and move it
  stepper.setTargetPositionInRevolutions(targetPosition);

  if (!stepper.motionComplete()) { // If stepper motor is not complete its move
    stepper.processMovement(); // Command stepper to move to target
  }
  // Update encoder count based on current position of stepper motor
    currentCount = map(stepper.getCurrentPositionInRevolutions(), -8, 8, -2000, 2000);
    encoderCount = currentCount;
    encoder.write(encoderCount);
  break;



    case SET_MODE:
      // Set encoder count to 0
      encoder.write(0);

      // Set current position of stepper motor to 0
      stepper.setCurrentPositionInRevolutions(0);
      break;

    case MANUAL_MODE: 
  // Read encoder count and map to target position of stepper motor
    // Limit encoder count to range of -2000 to 2000 steps
    encoderCount = encoder.read();
    if (encoderCount < -2000) {
    encoderCount = -2000;
    encoder.write(encoderCount);
  } else if (encoderCount > 2000) {
    encoderCount = 2000;
    encoder.write(encoderCount);
  }
  
  targetPosition = map(encoderCount, -2000, 2000, -8, 8);

  // Set target position of stepper motor and move it
  stepper.setTargetPositionInRevolutions(targetPosition);

  if (!stepper.motionComplete()) { // If stepper motor is not complete its move
    stepper.processMovement(); // Command stepper to move to target
  }
  break;
  }
}