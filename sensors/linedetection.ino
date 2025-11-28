// line detection sensors
const int SENSOR_LEFT_1 = A0;  
const int SENSOR_CENTER_1 = A1; 
const int SENSOR_RIGHT_1 = A2; 
const int SENSOR_LEFT_2 = A3;   
const int SENSOR_CENTER_2 = A4; 
const int SENSOR_RIGHT_2 = A5; 
const int THRESHOLD = 100; // need to be calibrated
int Vleft1;
int Vright1;
int Vcenter1;
int Vleft2;
int Vright2;
int Vcenter2;
int Vref1;
int Vref2;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Vleft1 = analogRead(SENSOR_LEFT_1);
  Vright1 = analogRead(SENSOR_RIGHT_1);
  Vcenter1 = analogRead(SENSOR_CENTER_1);
  Vref1 = (Vleft1 + Vright1 + Vcenter1)/3;
  Serial.println(Vref1);
  Vleft2 = analogRead(SENSOR_LEFT_2);
  Vright2 = analogRead(SENSOR_RIGHT_2);
  Vcenter2 = analogRead(SENSOR_CENTER_2);
  Vref2 = (Vleft2 + Vcenter2 + Vright2)/3; 
  Serial.println(Vref2);

  if(Vref1 - Vref2 > THRESHOLD){
    // line on right => turn left
    Serial.println("turn left");
  }
  else if(Vref2 - Vref1 > THRESHOLD){
    // line on left => turn right
    Serial.println("turn right");
  }
  // else if(Vref - Vcenter1 > THRESHOLD or Vref - Vcenter2 > THRESHOLD){
  //   // line on top => turn around 
  //   Serial.println("turn around");
  // }
  else{
    // go straight
    Serial.println("go straight");
  }
}
