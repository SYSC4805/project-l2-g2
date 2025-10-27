
// Motor Integration Test Code
// Controls left and right DC motors using L298N motor driver

// Motor A (Left)
int enA = 5;
int in1 = 8;
int in2 = 9;

// Motor B (Right)
int enB = 6;
int in3 = 10;
int in4 = 11;

void setup() {
  // Set motor control pins as output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
  Serial.println("Motor Integration Test Starting...");
}

// Move Forward
void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 200);
  analogWrite(enB, 200);
}

// Stop Motors
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Move Backward
void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 200);
  analogWrite(enB, 200);
}

// Turn Right 
void turnRight(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 200);
  analogWrite(enB, 200);
}

// Turn Left 
void turnLeft(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 200);
  analogWrite(enB, 200);
}


void loop() {
  Serial.println("Moving Forward");
  forward();
  delay(2000);
  stopMotors();
  delay(1000);

  Serial.println("Moving Backward");
  backward();
  delay(2000);
  stopMotors();
  delay(2000);

  Serial.println("Turning Right");
  turnRight();  
  delay(1000);
  stopMotors();
  delay(1000);

  Serial.println("Turning Left");
  turnLeft(); 
  delay(1000);
  stopMotors();
  delay(1000);
}