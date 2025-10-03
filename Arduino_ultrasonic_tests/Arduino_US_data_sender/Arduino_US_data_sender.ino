#include <ros.h>

#define ECHO 2
#define TRIGGER 3

void setup() {
  // put your setup code here, to run once:
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(9600);
}
float duration, distance;

void loop() {
  // put your main code here, to run repeatedly:
	digitalWrite(TRIGGER, LOW);  
	delayMicroseconds(2);  
	digitalWrite(TRIGGER, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(TRIGGER, LOW);  

  duration = pulseIn(ECHO, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
}
