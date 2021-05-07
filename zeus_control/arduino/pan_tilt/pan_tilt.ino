
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
Servo servo1;
Servo servo2;
int servo_1_cmd, old_servo_1_cmd;
int servo_2_cmd, old_servo_2_cmd;

ros::NodeHandle nh;

void commandCb( const std_msgs::Int16MultiArray& cmd_msg){
    servo_1_cmd = cmd_msg.data[0];
    servo_2_cmd = cmd_msg.data[1];
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("/zeus_control/pantilt", &commandCb );


void setup() {
  // Init servos
  servo1.attach(10);
  servo2.attach(11);
  servo1.write(90);
  servo2.write(90);
  Serial.begin(57600);  

  // ROS stuff
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  if (servo_1_cmd != old_servo_1_cmd){
    servo1.write(servo_1_cmd);
  }
  if (servo_2_cmd != old_servo_2_cmd){
    servo2.write(servo_2_cmd);
  }
  nh.spinOnce();
}
