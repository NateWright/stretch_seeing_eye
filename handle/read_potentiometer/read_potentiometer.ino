// /*
//   AnalogReadSerial
//   https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
// */
#include <ros.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle nh;

std_msgs::UInt32 msg;
ros::Publisher chatter("/stretch_seeing_eye/handle_reading", &msg);


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  msg.data = analogRead(A0);
  chatter.publish( &msg );
  nh.spinOnce();
  delay(100);
}