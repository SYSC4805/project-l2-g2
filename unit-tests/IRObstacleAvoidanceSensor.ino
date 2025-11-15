#define IRpin 2 
void setup() { 
	// put your setup code here, to run once: 
  Serial.begin(9600);
  pinMode(IRpin,INPUT); 
} 
void loop() { 
	// put your main code here, to run repeatedly: 
	int IRread = digitalRead(IRpin); 
if(IRread == 0){ 
  Serial.println("obstacle detected");
} 
else{
  Serial.println("no obstacle");
}
} 
