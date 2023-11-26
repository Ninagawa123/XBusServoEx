// * cavle connection *
//   XBusServo     -  ESP32
//   outside black -  GND
//   center black  -  5V or (POWER)
//   outside white -  pin 17

#include <XBusServoEx.h>

#define  kMaxServoNum    1       // numbers of XBus Servos 1 to 50
#define  kDirPinNum      -1      // tx only mode

XBusServoEx      myXBusServo(kDirPinNum, kMaxServoNum); 
unsigned int   servoValue = 32767;// center of XBusServo value(0x7FFF)
float radiansval = 0.0; 
float radiansIncrement = 0.03;

void setup()
{
  myXBusServo.begin2();
  myXBusServo.addServo(0x01, kXbusServoNeutral);    // servoID
}

void sendPacket()
{
  myXBusServo.sendChannelDataPacket2();
}


void loop()
{
  // make a sign curve
  radiansval += radiansIncrement;  
  radiansval = (radiansval > 2 * PI) ? 0 : radiansval; 

  // set servo value
  servoValue = 32767 + int(sin(radiansval) * 30426); 
  myXBusServo.setServo(1, servoValue); //set servo value
  myXBusServo.sendChannelDataPacket2(); //send command to servo
  delay(10); 
}

