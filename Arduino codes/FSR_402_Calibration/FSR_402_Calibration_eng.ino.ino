const int fsrPin = A0; // Analog pin
float calibrationFactor = 0.00511; // Calibration factor - based on your test and adjustments

int fsrBaseline = 0; // Baseline (tare) value calculated in setup

void setup() {
  Serial.begin(9600);
  Serial.println("Calibrating the sensor...");

  // Read the sensor multiple times to get an average and use it as the baseline
  const int numReads = 200;
  long sum = 0;
  for (int i = 0; i < numReads; i++) {
    sum += analogRead(fsrPin);
    delay(30);
  }
  fsrBaseline = sum / numReads; // Average value read
  Serial.print("Baseline value is: ");
  Serial.println(fsrBaseline);
  Serial.println("Calibration done. Starting main loop...");
  delay(1000);
}

void loop() {
  // Read the sensor value
  int fsrReading = analogRead(fsrPin);

  // Subtract the baseline value to zero out the sensor in the absence of force
  int fsrOffset = fsrReading - fsrBaseline;
  

  // Calculate the weight (grams) from the baseline-compensated value
  float weightGram = fsrOffset * calibrationFactor;

  // Display
  Serial.print("ADC Raw: ");
  Serial.print(fsrReading);
  Serial.print(" | Offset: ");
  Serial.print(fsrOffset);
  Serial.print(" | Weight: ");
  Serial.print(weightGram);
  Serial.println(" g");

  delay(200);
}
