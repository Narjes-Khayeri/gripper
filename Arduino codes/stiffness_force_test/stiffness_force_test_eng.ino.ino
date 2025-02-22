#include <Servo.h>

const int fsrPin = A0;         // FSR sensor pin
int fsrBaseline = 0;           // Baseline value obtained after calibration
const float fsrFactor = 0.00511; // Conversion factor from ADC difference to Newton (based on experiment)

// Function to store and sort ADC data
// and get the average after removing extreme samples
float getStableForce() {
  const int numSamples = 50;   // Number of samples to take
  int readings[numSamples];

  // Sampling
  for(int i = 0; i < numSamples; i++){
    readings[i] = analogRead(fsrPin);
    delay(5); // Interval between each sample (5ms)
  }

  // Sorting readings using a simple method (bubble sort or any preferred method)
  for(int i = 0; i < numSamples - 1; i++){
    for(int j = 0; j < numSamples - i - 1; j++){
      if(readings[j] > readings[j+1]) {
        int temp = readings[j];
        readings[j] = readings[j+1];
        readings[j+1] = temp;
      }
    }
  }

  // Remove 2 minimum and 2 maximum samples
  // (to reduce the impact of noise and outlier data)
  long sum = 0;
  int count = 0;
  for(int i = 4; i < numSamples - 2; i++){
    sum += readings[i];
    count++;
  }

  // Average of middle samples
  float avgReading = sum / (float)count;

  // Subtract baseline value
  float rawValue = avgReading - fsrBaseline;
  if(rawValue < 0) {
    rawValue = 0;
  }

  // Calculate force (in Newton)
  float force = rawValue * fsrFactor;
  return force;
}

Servo myServo;              // Servo object
int servoAngle = 0;         // Servo angle (0 to 180 degrees)

// Variables for stiffness test
bool stiffnessTestDone = false;   
float initialForce = 0.0;   // Initial force when reaching 0.1N
float finalForce = 0.0;     // Force after an additional 4 degrees of movement
float stiffnessValue = 0.0; // Stiffness value
float deltaForce = 0.0;     // Force difference (finalForce - initialForce)

// =========================
// Sensor calibration function
// =========================
void calibrateFSR(int pin) {
  Serial.println("Calibrating the sensor...");

  // Read the sensor multiple times to get an average and use it as baseline
  const int numReads = 200;
  long sum = 0;
  for (int i = 0; i < numReads; i++) {
    sum += analogRead(pin);
    delay(30);
  }
  fsrBaseline = sum / numReads; // Average value read
  Serial.print("Baseline value is: ");
  Serial.println(fsrBaseline);
  Serial.println("Calibration done. Starting main loop...");
  delay(1000);
}

void setup() {
  Serial.begin(9600);   // Begin serial communication
  myServo.attach(9);    // Attach servo to digital pin 9

  // First, calibrate the sensor
  calibrateFSR(fsrPin);

  // Set the gripper to fully open (zero angle)
  servoAngle = 0;
  myServo.write(servoAngle);
}

void loop() {

  // Until stiffness test is completed:
  if (!stiffnessTestDone) {

    // Move from angle 0 to 180; for each step:
    // 1) Close the servo by 1 degree
    // 2) Read the force using getStableForce() (about ~250ms each time)
    // 3) Print the force
    // 4) If force >= 0.1N, stop
    for (servoAngle = 0; servoAngle <= 180; servoAngle++) {
      myServo.write(servoAngle);

      // Read the stable force using the function above
      float force = getStableForce();

      // Report angle and force with decimal precision
      Serial.print("Angle: ");
      Serial.print(servoAngle);
      Serial.print(" degrees, Force: ");
      Serial.print(force, 3); // Three decimal places
      Serial.println(" Newton");

      // Check if the force threshold is reached
      if (force >= 0.1) {
        initialForce = force; // Initial force at the threshold
        break; // Exit the loop
      }
    }

    // Now close the servo by an additional 4 degrees
    servoAngle += 4;
    if (servoAngle > 180) {
      servoAngle = 180; // Prevent going beyond 180 degrees
    }
    myServo.write(servoAngle);

    // Read the force again
    finalForce = getStableForce();

    // Calculate stiffness
    deltaForce = finalForce - initialForce;
    float displacement = 0.001188; // Meters (displacement between the two fingers at 4 degrees)
    stiffnessValue = deltaForce / displacement;

    // Report stiffness and force difference
    Serial.println("==================================");
    Serial.print("Initial Force (N): ");
    Serial.println(initialForce, 3);
    Serial.print("Final Force (N): ");
    Serial.println(finalForce, 3);
    Serial.print("Force Difference (DeltaF) = ");
    Serial.print(deltaForce, 3);
    Serial.println(" N");
    Serial.print("Object Stiffness (N/m) = ");
    Serial.print(stiffnessValue, 3);
    Serial.println(" N/m");
    Serial.println("==================================");

    // Test completion message
    Serial.println("Stiffness test is complete. Waiting for serial commands (+ or -).");

    // Now the test is finished, and we will enter manual control mode
    stiffnessTestDone = true;
  }
  else {
    // In this phase, the test is complete and we are only waiting for serial commands
    // If '+' is received -> close by one degree
    // If '-' is received -> open by one degree
    // Then read and report the force
    if (Serial.available() > 0) {
      char command = Serial.read();

      if (command == '+') {
        servoAngle++;
        if (servoAngle > 180) {
          servoAngle = 180; // Prevent going beyond 180
        }
        myServo.write(servoAngle);

        // Read and display the force
        float force = getStableForce();
        Serial.print("Angle: ");
        Serial.print(servoAngle);
        Serial.print(" degrees, Force: ");
        Serial.print(force, 3);
        Serial.println(" Newton");
      }
      else if (command == '-') {
        servoAngle--;
        if (servoAngle < 0) {
          servoAngle = 0; // Prevent going below 0
        }
        myServo.write(servoAngle);

        // Read and display the force
        float force = getStableForce();
        Serial.print("Angle: ");
        Serial.print(servoAngle);
        Serial.print(" degrees, Force: ");
        Serial.print(force, 3);
        Serial.println(" Newton");
      }
      // If any other command is received, it will be ignored for now
    }
  }
}
