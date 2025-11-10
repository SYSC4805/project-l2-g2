void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("O1: ");
  Serial.print(analogRead(5)* (5.0 / 1023.0));
  Serial.print(" | O3: ");
  Serial.print(analogRead(7)* (5.0 / 1023.0));
  Serial.print(" | O2: ");
  Serial.println(analogRead(6)* (5.0 / 1023.0));

  delay(500);
}
