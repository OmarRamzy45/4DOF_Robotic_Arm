#include <ros.h>
#include <Servo.h>
#include <std_msgs/Float64MultiArray.h>

//rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200

ros::NodeHandle nh;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;

float servo_angle[5] = {0, 90, 90, 90, 180};


void JointStates_callback(const std_msgs::Float64MultiArray& joint_angle)
{
  servo_angle[1] = joint_angle.data[1];
  servo_angle[2] = joint_angle.data[2];
  servo_angle[3] = joint_angle.data[3];
  servo_angle[4] = joint_angle.data[4];
}

ros::Subscriber<std_msgs::Float64MultiArray> JointStates_subscriber("joint_angles", JointStates_callback);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(JointStates_subscriber);
    
  servo_1.attach(3);
  servo_2.attach(5);
  servo_3.attach(6);
  servo_4.attach(9);
  servo_1.write(servo_angle[1]);
  servo_2.write(servo_angle[2]);
  servo_3.write(servo_angle[3]);
  servo_4.write(servo_angle[4]);
}

void loop()
{
  servo_1.write(servo_angle[1]);
  servo_2.write(servo_angle[2]);
  servo_3.write(servo_angle[3]);
  servo_4.write(servo_angle[4]);
  nh.spinOnce();
  delay(1);
}
