
#define enA 3
#define in1 6
#define in2 7 //to change direction of rotation   
#define inSensor 2


int count = 0;
//float omega = 0;
//float actual_velocity=0.8;
int prev_time=millis();
int elapsed_time=0;
int rotDirection = 0;

//float dt = 0.12 ; //[s] time tick
float kp = 0.01 ;//Proportional Gain
float desired_rpm = 500;
float actual_rpm = 0 ;
float error_rpm =0;
float previous_pwm = 0;
float new_pwm = 0;
int prev_sensor = 0; 
//float a = 0;



void setup() {
  Serial.begin (9600); // Serial output with 9600 bps
  pinMode(inSensor, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 20);
 
}

void loop() {
   
   
  int Digital;
  Digital = digitalRead(inSensor);
  
  
    
   if(Digital==1&& prev_sensor==0){
     
   elapsed_time = millis() - prev_time;
   Serial.print("elapsed_time   : ");
   Serial.println(elapsed_time);
   
   float actual_rpm = (1/(elapsed_time*0.001))*(60) ;  //Speed (RPM) = (Pulses per Revolution / Time) x 60
   Serial.print("actual_rpm   :  ");
   Serial.println(actual_rpm);
   
   /*omega = (2*3.14*rpm)/60;
   actual_velocity = omega*0.03;  //radius of wheel
   Serial.print("actual velocity   :   ");
   Serial.println (actual_velocity);*/
   
   error_rpm = desired_rpm - actual_rpm;
   new_pwm = previous_pwm  + (elapsed_time*(error_rpm*kp)/100);


   
   /*a = kp * (desired_velocity - actual_velocity);
   actual_velocity = actual_velocity +(a * dt);
   pwm = actual_velocity*20 / 0.5 ; */
   
     
   new_pwm = constrain(new_pwm,20,150);
   analogWrite(enA,new_pwm);
   previous_pwm = new_pwm;
   
  
   Serial.print("new_pwm   :   ");
   Serial.println (new_pwm);
   delay(50);
   prev_time = millis();
   
   
   
   Serial.println ("------------------------------------------------------");
   }
   prev_sensor= Digital; 
  
}
