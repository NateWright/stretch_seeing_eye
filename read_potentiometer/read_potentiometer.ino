// /*
//   AnalogReadSerial

//   Reads an analog input on pin 0, prints the result to the Serial Monitor.
//   Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
//   Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

//   This example code is in the public domain.

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