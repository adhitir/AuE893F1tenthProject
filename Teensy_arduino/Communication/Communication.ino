
#include <ros.h>
//#include <std_msgs/Bool.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo servo;
Servo esc;

int pwm_center_value = 90;  //  15% duty cycle
int pwm_lowerlimit = 60;    //  10% duty cycle
int pwm_upperlimit = 120;   //  20% duty cycle

void messageDrive( const geometry_msgs::Twist& input )
{
  int pwm_drive = input.linear.x +90;
  int pwm_angle = input.angular.z +90;

  if(pwm_drive < pwm_lowerlimit)
  {
    esc.write(pwm_lowerlimit);    //  Safety lower limit        
  }
  else if(pwm_drive > pwm_upperlimit)
  {
    esc.write(pwm_upperlimit);    //  Safety upper limit
  }
  else
  {
    esc.write(pwm_drive);     //  Incoming data                    
  }
 
  if(pwm_angle < pwm_lowerlimit)
  {
    servo.write(pwm_lowerlimit);    //  Safety lower limit        
  }
  else if(pwm_angle > pwm_upperlimit)
  {
    servo.write(pwm_upperlimit);    //  Safety upper limit
  }
  else
  {
    servo.write(pwm_angle);     //  Incoming data                    
  }
  
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &messageDrive);

void setup() {
  servo.attach(5);
  esc.attach(6);
  nh.initNode();
  nh.subscribe(cmd_vel);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
