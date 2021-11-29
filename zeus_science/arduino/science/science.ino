/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: continuous rotation. More info: https://www.makerguides.com */
// Define stepper motor connections:
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#define upPin 2
#define downPin1 3
#define downPin2 4
#define dirPin 5
#define stepPin 6
#define EnableStepper 8
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
bool StepperSignal;
int incomingCommand;
int EnableStepperSignal;

ros::NodeHandle nh;

void commandCb( const std_msgs::Int16& cmd_msg){
    incomingCommand = cmd_msg.data;
}

ros::Subscriber<std_msgs::Int16> sub("/zeus_science/command", &commandCb );


void setup() {
  // Declare pins as outputs:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(EnableStepper, OUTPUT);
  
  // Declare pins as inputs:
  pinMode(upPin, INPUT);
  pinMode(downPin1, INPUT);
  pinMode(downPin2, INPUT);
  pinMode(13, INPUT);
  
  // Init states
  incomingCommand=0;
  digitalWrite(EnableStepper,LOW);

  // Init servos
  servo1.attach(10);
  servo2.attach(11);
  servo3.attach(12);
  servo4.attach(13);

  // Begin serial port
  Serial.begin(57600);

  nh.initNode();
  nh.subscribe(sub);
}
void loop() {
  
    switch (incomingCommand){
      case 0:
        break;
        
      case 1: 
      // open bottom left
        servo1.write(0);
        break;
      
      case 2:
      // close bottom left
        servo1.write(175);
        break;
        
      case 3:
      // open top right
        servo2.write(0);
        break;
        
      case 4:
      // close top right
        servo2.write(175);
        break;
      
      case 5:
      // close bottom right
        servo3.write(0);
        break;
        
      case 6:
      // open bottom right
        servo3.write(175);
        break;
        
      case 7:
      // close top left
        servo4.write(0);
        break;
        
      case 8:
      // open top left
        servo4.write(175);
        break;
        
      case 9:
      // up
        if (digitalRead(upPin)== LOW){
          digitalWrite(EnableStepper,LOW);
          // Set the spinning direction CW/CCW:
          digitalWrite(dirPin, LOW);
          // These four lines result in 1 step:
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(200);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(200);}
        else{
          digitalWrite(stepPin, LOW);
          }
          break;
      case 10:
      // down
        if (digitalRead(downPin1)==LOW || digitalRead(downPin2)==LOW){
          digitalWrite(EnableStepper,LOW);
          // Set the spinning direction CW/CCW:
          digitalWrite(dirPin, HIGH);
          // These four lines result in 1 step:
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(200);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(200);}
        else{
          digitalWrite(EnableStepper,HIGH);    
          }
          break;
        }
        

  nh.spinOnce();
}
