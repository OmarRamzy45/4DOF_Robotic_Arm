#include <SimpleTimer.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h> 
#include <std_msgs/Float64.h> 

#define pwm 6
#define inl 7
#define in2 8
#define EncoderPinA 2 //Encoder Channel A 
#define EncoderPinB 3 //Encoder Channel B 

ros::NodeHandle nh;
std_msgs::Float64 position_feedback;

//Timer Object 
SimpleTimer timer;

//Timer interval Length in 100 ms 
const int timerInterval = 100L;

//Output PWM control signal for motor 
unsigned char pwmOutput = 0; 
unsigned char outMax = 150, outMin = 35;
//Input Speed in RPM 
float motor_position = 0.0;
float SpeedRPM = 0.0;
//Required Speed in RPM 
int Setpoint = 0;   
//PID tuning parameters 
float Kp = 2.11, Ki = 0.00017, Kd = 0.00057; // 30 degrees constants
//float Kp = 2.38, Ki = 0.0006, Kd = 0.00008; // 30 degrees constants
float iTerm = 0.0, dTerm = 0.0, prevIntegration = 0.0;
float currentError = 0.0, prevError = 0.0; 

//Encoder Counter
volatile signed long encoderCount = 0;
//Encoder Pin B pervious Value
bool perviousPinB = LOW;
//Encoder Pulse Per Gearbox output Revolution
const float encoderPPR = 800.0; 

void JointStates_callback(const std_msgs::Float64& joint_angle)
{
  Setpoint = joint_angle.data;
}

ros::Subscriber<std_msgs::Float64> JointStates_subscriber("setpoint", JointStates_callback);
ros::Publisher position_publisher("feedback_position", &position_feedback);

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm,OUTPUT);
  pinMode(inl, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(EncoderPinA, INPUT_PULLUP);
  pinMode(EncoderPinB, INPUT_PULLUP);

  attachInterrupt(0, EncoderEvent, CHANGE);
  attachInterrupt(1, EncoderEvent, CHANGE);

  //Setting Timer to read motor Speed every 100 msec
  timer.setInterval(timerInterval, timerRoutine);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(JointStates_subscriber);
  nh.advertise(position_publisher);

}

int EncoderEvent() 
{
  /* the function works to compare between the state of each encoder in a two channel encoder to know which direction the motor is rotating */

  if (digitalRead(EncoderPinA) == HIGH)
  {
    if (digitalRead(EncoderPinB) == LOW && perviousPinB == LOW) 
    {
      // clockwise direction
      encoderCount++;
      // motorDirection = 'CW';
      perviousPinB = LOW;
    }
    else if (digitalRead(EncoderPinB) == HIGH && perviousPinB == LOW) 
    {
      // clockwise direction
      encoderCount++;
      // motorDirection = 'CW';
      perviousPinB = HIGH;
    }
    else if (digitalRead(EncoderPinB) == LOW && perviousPinB == HIGH) 
    {
      // counter-clockwise direction
      encoderCount--;
      // motorDirection = 'CCW';
      perviousPinB = LOW;
    } 
    else if (digitalRead(EncoderPinB) == HIGH && perviousPinB == HIGH)
    {
      // counter-clockwise direction
      encoderCount--;
      // motorDirection = 'CCW';
      perviousPinB = HIGH;
    }
  }
  else 
  { // if (digitalRead(EncoderPinA) == LOW)

    if (digitalRead(EncoderPinB) == LOW && perviousPinB == LOW) 
    {
      // counter-clockwise direction
      encoderCount--;
      // motorDirection = 'CCW';
      perviousPinB = LOW;
    }
    else if (digitalRead(EncoderPinB) == HIGH && perviousPinB == LOW) 
    {
      // counter-clockwise direction
      encoderCount--;
      // motorDirection = 'CCW';
      perviousPinB = HIGH;
    }
    else if (digitalRead(EncoderPinB) == LOW && perviousPinB == HIGH) 
    {
      // clockwise direction
      encoderCount++;
      // motorDirection = 'CW';
      perviousPinB = LOW;
    } 
    else if (digitalRead(EncoderPinB) == HIGH && perviousPinB == HIGH)
    {
      // clockwise direction
      encoderCount++;
      // motorDirection = 'CW';
      perviousPinB = HIGH;
    }
  }
}

void timerRoutine ()
{
  motor_position = (encoderCount/encoderPPR)*360;
  currentError = Setpoint - motor_position;

  iTerm = prevIntegration + 0.5 * Ki * (currentError + prevError) * timerInterval;
  dTerm = Kd * (currentError - prevError) / timerInterval;

  pwmOutput = Kp * currentError + iTerm + dTerm;
  position_feedback.data = motor_position;
  position_publisher.publish(&position_feedback);
  

  if (pwmOutput > outMax) 
  {
    pwmOutput = outMax;
  }
  
  else if (pwmOutput < outMin) 
  {
    pwmOutput = outMin;
  }

  if (currentError > 0)
  {
  analogWrite(pwm, pwmOutput);
  digitalWrite(inl, HIGH);
  digitalWrite(in2, LOW);
  }
  
  if (currentError < 0)
  {
  analogWrite(pwm, pwmOutput);
  digitalWrite(inl, LOW);
  digitalWrite(in2, HIGH);
  }

  prevError = currentError;
  prevIntegration = iTerm;
}

void loop() 
{
  timer.run();
  nh.spinOnce();
  delay(1);
}
