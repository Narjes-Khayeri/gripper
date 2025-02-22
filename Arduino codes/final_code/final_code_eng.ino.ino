#include <Servo.h>
#include <LiquidCrystal.h>

// --------------- Pins -----------------
const int buttonPin = 8;  // Button for start
const int servoPin  = 9;  // Servo
const int fsrPin    = A0; // FSR sensor
const int ledPin    = 13; // LED for blinking

// LCD 16x2 in 4-bit mode
// Order: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// -------------------- Base Variables --------------------
int   fsrBaseline = 0;       // Baseline after calibration
float fsrFactor   = 0.00511; // Conversion factor from ADC difference to Newton

// Mechanical information
float baseDistance      = 56.0;  // Distance between fingers at 0 degrees
float distancePer175Deg = 52.0;  // Change in distance for 175 degrees

// Servo
Servo myServo;
int servoAngle = 0;

// PID Variables
float Kp = 5.0;
float Ki = 0.0;
float Kd = 0.0;

float integralError = 0.0;
float lastError     = 0.0;
unsigned long lastPIDTime = 0;

// Stiffness and target force
float stiffnessValue   = 0.0;
float initialForce     = 0.0;
float finalForce       = 0.0;
float targetForce      = 0.0;
float lowerBound       = 0.0;
float upperBound       = 0.0;
float finalHoldingForce= 0.0; // Final force that remains within the bounds after PID

// State machine
int state = 0;

/*
   state 0: Waiting for button press
   state 1: Long calibration
   state 2: Stiffness test (holding object until 0.1N + 4 degrees) => only show angle and force while closing
   state 3: Running PID until reaching target force (or timeout) => display "PID in progress..." on LCD
   state 4: Hold for 10 seconds (blink in last 3 seconds) => display stiffness, target force, and final force
   state 5: Open gripper
   state 6: Wait 5 seconds + quick calibration in first 2 seconds + blink in last 3 seconds + return to state=2
*/

// -------------------- Helper Functions --------------------

// Calculate distance between fingers based on servo angle
float getGripperDistance(int angle){
  if(angle > 175) angle = 175;
  return baseDistance - (distancePer175Deg/175.0) * angle;
}

// Read stable FSR (50 samples + remove 4 minimum and 4 maximum)
float getStableForce() {
  const int numSamples = 50;
  int readings[numSamples];

  for(int i=0; i<numSamples; i++){
    readings[i] = analogRead(fsrPin);
    delay(5);
  }

  // Simple sorting
  for(int i=0; i<numSamples-1; i++){
    for(int j=0; j<numSamples-1-i; j++){
      if(readings[j] > readings[j+1]){
        int temp = readings[j];
        readings[j] = readings[j+1];
        readings[j+1] = temp;
      }
    }
  }

  // Remove 4 minimum and 4 maximum values
  long sum = 0;
  int count = 0;
  for(int i=4; i<numSamples-4; i++){
    sum += readings[i];
    count++;
  }

  float avg = sum / (float)count;
  float rawValue = avg - fsrBaseline;
  if(rawValue < 0) rawValue = 0;

  float force = rawValue * fsrFactor;
  return force;
}

// Long calibration
void calibrateFSR() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating...");
  
  const int numReads = 200;
  long sum = 0;
  for(int i = 0; i < numReads; i++){
    sum += analogRead(fsrPin);
    delay(30);
  }
  fsrBaseline = sum / numReads;

  lcd.setCursor(0,1);
  lcd.print("Done Base=");
  lcd.print(fsrBaseline);
  delay(1000);
}

// Quick calibration
void calibrateFSRQuick() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Quick Cal...");
  
  const int numReads = 50;
  long sum = 0;
  for(int i=0; i<numReads; i++){
    sum += analogRead(fsrPin);
    delay(20);
  }
  fsrBaseline = sum / numReads;

  lcd.setCursor(0,1);
  lcd.print("Done Base=");
  lcd.print(fsrBaseline);
  delay(800);
}

// Blink LED (3 times, each time 0.5s on + 0.5s off)
void blinkLed3Times(){
  for(int i=0; i<3; i++){
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

// Simple display of "servo angle" and "force" on LCD (two lines)
void displayAngleForce(int angle, float force){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Angle=");
  if(angle<100) lcd.print("0");
  if(angle<10)  lcd.print("0");
  lcd.print(angle);

  lcd.setCursor(0,1);
  lcd.print("Force=");
  lcd.print(force,2);
}

// Display stiffness, target force, and final force (three parameters)
void displayStiffnessTargetFinal(float stiff, float target, float finalF){
  lcd.clear();
  // Due to limited LCD space, we write it briefly
  // First line: St=? T=?
  lcd.setCursor(0,0);
  lcd.print("St=");
  lcd.print(stiff,2);
  lcd.print(" T=");
  lcd.print(target,2);

  // Second line: F=?
  lcd.setCursor(0,1);
  lcd.print("F=");
  lcd.print(finalF,2);
}

// ----------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------
void setup(){
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  lcd.begin(16,2); // LCD 16x2

  myServo.attach(servoPin);
  myServo.write(0);

  // First, enter state 0: Waiting for button press
  state = 0;
}

// ----------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------
void loop(){

  switch(state){

    // ------------------------------
    // state 0: Waiting for button press
    // ------------------------------
    case 0:
    {
      // Message on LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Press Button");
      lcd.setCursor(0,1);
      lcd.print("To Start...");

      // Stay here until button is released (HIGH)
      while(digitalRead(buttonPin) == HIGH){
        // Do nothing
      }
      delay(200); // Debounce effect removal
      // When button is pressed:
      state = 1;
    }
    break;

    // ------------------------------
    // state 1: Calibration
    // ------------------------------
    case 1:
    {
      calibrateFSR();
      servoAngle = 0;
      myServo.write(servoAngle);

      // Move to stiffness test
      state = 2;
    }
    break;

    // ------------------------------
    // state 2: Stiffness test (close to 0.1N)
    // ------------------------------
    case 2:
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Meas. Stiffness");

      // 1) Close from angle 0 to 180 until force reaches 0.1N
      for(servoAngle=0; servoAngle<=180; servoAngle++){
        myServo.write(servoAngle);
        float force = getStableForce();

        // As requested: only show angle and force
        displayAngleForce(servoAngle, force);

        if(force >= 0.1){
          initialForce = force;
          break;
        }
      }

      // +4 degrees
      servoAngle += 4;
      if(servoAngle>180) servoAngle=180;
      myServo.write(servoAngle);

      float f2 = getStableForce();
      displayAngleForce(servoAngle, f2);
      finalForce = f2;

      float deltaF = finalForce - initialForce;
      float displacement = 0.001188; // Displacement for 4 degrees
      stiffnessValue = deltaF / displacement;

      delay(800); // Small pause

      state = 3;
    }
    break;

    // ------------------------------
    // state 3: Running PID
    // ------------------------------
    case 3:
    {
      // Simple page for awareness
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("PID in progress");

      // Target force = stiffness × 0.0024
      targetForce = stiffnessValue * 0.0024;
      lowerBound  = 0.9 * targetForce;
      upperBound  = 1.1 * targetForce;

      // Prepare PID
      integralError = 0.0;
      lastError = 0.0;
      lastPIDTime = millis();

      // Maximum allowed time
      const unsigned long maxPidTime = 4000; // e.g., 4 seconds
      unsigned long pidStart = millis();

      bool inRange = false;
      finalHoldingForce = 0.0; // Initial zero

      while(true){
        unsigned long now = millis();
        if(now - pidStart > maxPidTime){
          // Time is up
          break;
        }

        float dt = (now - lastPIDTime)/1000.0;
        lastPIDTime = now;

        float force = getStableForce();
        float error = targetForce - force; 

        // PID
        integralError += error * dt;
        float deriv = (error - lastError)/dt;
        lastError = error;

        float output = Kp*error + Ki*integralError + Kd*deriv;

        servoAngle += (int)output;
        if(servoAngle < 0) servoAngle=0;
        if(servoAngle > 180) servoAngle=180;
        myServo.write(servoAngle);

        // If force enters the range, exit
        if(force >= lowerBound && force <= upperBound){
          inRange = true;
          finalHoldingForce = force; // Force when it first entered range
          break;
        }
      }

      if(!inRange){
        // If it failed to enter the range
        finalHoldingForce = getStableForce(); // The value at the end
      }

      state = 4; // Move to holding
    }
    break;

    // ------------------------------
    // state 4: Holding for 10 seconds
    // ------------------------------
    case 4:
    {
      // Throughout the 10s, show stiffness, target force, and final force
      // Write at the beginning and do not change until the end of 10s
      displayStiffnessTargetFinal(stiffnessValue, targetForce, finalHoldingForce);

      unsigned long startTime = millis();
      unsigned long holdDuration = 10000; 
      bool blinkedYet = false;

      while(true){
        unsigned long elapsed = millis() - startTime;
        if(elapsed >= holdDuration) break; // End of 10s

        // Blink in last 3 seconds
        if(elapsed >= 7000 && !blinkedYet){
          blinkLed3Times();
          blinkedYet = true;
        }
        delay(50);
      }

      state = 5;
    }
    break;

    // ------------------------------
    // state 5: Open gripper
    // ------------------------------
    case 5:
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Releasing...");
      servoAngle = 0;
      myServo.write(servoAngle);
      delay(500);

      state = 6;
    }
    break;

    // ------------------------------
    // state 6: 5s wait + quick calibration + blink
    // ------------------------------
    case 6:
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Wait 5s...");
      
      unsigned long startTime = millis();
      bool calDone   = false;
      bool blinkDone = false;

      while(true){
        unsigned long elapsed = millis() - startTime;
        if(elapsed >= 5000) break;

        // First 2s: quick calibration
        if(elapsed < 2000 && !calDone){
          calibrateFSRQuick();
          calDone = true;
        }
        // Last 3s: blink
        if(elapsed >= 2000 && !blinkDone){
          blinkLed3Times();
          blinkDone = true;
        }
        delay(50);
      }

      // End of cycle, go back to state=2 for the next object
      state = 2;
    }
    break;
  } // switch
}
