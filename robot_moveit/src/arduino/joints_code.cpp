#include <ros.h>
#include <servo.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;
Servo servo_1;
Servo servo_2;
Servo servo_3;


// array of joint angles
float servo_angle[6] = {0,0,0,0,0,0};

// callback function for joint angles
void cmd_cb(const sensor_msgs::JointState& joint_angle)
{
  servo_angle[1] = joint_angle.position[1] * (PI / 180.0);
  servo_angle[2] = joint_angle.position[2] * (PI / 180.0);
  servo_angle[3] = joint_angle.position[3] * (PI / 180.0);
  servo_1.write(servo_angle[1]);
  servo_2.write(servo_angle[2]);
  servo_3.write(servo_angle[3]);
 }

ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", cmd_cb);

void setup()
{
  servo_1.attach(3);
  servo_2.attach(5);
  servo_3.attach(6);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}