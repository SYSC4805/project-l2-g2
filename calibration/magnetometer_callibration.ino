#include <Wire.h>
#include <LIS3MDL.h>

LIS3MDL mag;

// Variables to store min/max readings
int16_t xMin = 32767;
int16_t xMax = -32768;
int16_t yMin = 32767;
int16_t yMax = -32768;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!mag.init()) {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  mag.enableDefault();

  Serial.println("Magnetometer Calibration Ready.");
  Serial.println("--------------------------------------------------");
  Serial.println("Step 1: Rotate the robot in full circles on the floor.");
  Serial.println("Step 2: Watch the 'Offset' values below.");
  Serial.println("Step 3: When values stabilize, copy them to your main code.");
  Serial.println("--------------------------------------------------");
  delay(2000);
}

void loop() {
  mag.read();

  // 1. Read raw values
  int16_t x = mag.m.x;
  int16_t y = mag.m.y;

  // 2. Find Min and Max
  if (x < xMin) xMin = x;
  if (x > xMax) xMax = x;

  if (y < yMin) yMin = y;
  if (y > yMax) yMax = y;

  // 3. Calculate Offsets (Midpoint)
  // Offset = (Max + Min) / 2
  float xOffset = (xMax + xMin) / 2.0;
  float yOffset = (yMax + yMin) / 2.0;

  // 4. Report to Serial Monitor
  Serial.print("Raw X: "); Serial.print(x);
  Serial.print("  Raw Y: "); Serial.print(y);
  
  Serial.print("  ||  MIN X: "); Serial.print(xMin);
  Serial.print("  MAX X: "); Serial.print(xMax);
  Serial.print("  MIN Y: "); Serial.print(yMin);
  Serial.print("  MAX Y: "); Serial.print(yMax);

  Serial.print("  ||  >>> USE THESE OFFSETS >>> X: "); 
  Serial.print(xOffset, 1);
  Serial.print(", Y: "); 
  Serial.println(yOffset, 1);

  delay(100);
}
