#include <Servo.h>


#define enA 3
#define in1 6
#define in2 7
#define servoPin 9

Servo myservo;  // create servo object to control a servo

String data = ""; // initialize data variable to empty string


void setup() {
  
  Serial.begin(9600);
  myservo.attach(servoPin);  // attaches the servo on pin 1 to the servo object
 pinMode(enA, OUTPUT);
 pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);  
   pinMode(servoPin, OUTPUT);

  //Set initial rotation direction = 0 
 digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  analogWrite(enA, 0);  // set initial duty cycle to 0
     delay(30);
}

void loop() {
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    Serial.print("You sent me: ");
    Serial.println(data);
    
  if (data == "FORWARD") {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 30);
     delay(20);
  } else if (data == "BACKWARD") {
   digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
    analogWrite(enA, 30);
     delay(20);
  } else if (data == "RIGHT") {
    myservo.write(135);
     delay(20);
  } else if (data == "LEFT") {
    myservo.write(45);
     delay(20);
  }
  else if (data == "stop") {
    analogWrite(enA, 0);
  }
  
  
  }

}

