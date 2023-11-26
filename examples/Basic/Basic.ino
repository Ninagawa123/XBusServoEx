#include <MsTimer2.h>
#include <XBusServoEx.h>

#define  kMaxServoNum    16       // 1 - 50
#define  kDirPinNum      2        // pin number for direction

XBusServo      myXBusServo(kDirPinNum, kMaxServoNum);
unsigned int   servoValue;


void setup()
{
  servoValue = kXbusServoNeutral;
  
  myXBusServo.begin();
  myXBusServo.addServo(0x01, kXbusServoNeutral);    // add first servo with channelID = 0x01
  // you can add more servo at here until kMaxServoNum.
  
  MsTimer2::set(kXBusInterval, sendPacket);
  MsTimer2::start();
}


// This is the handler to keep to send channel command packet with 14mSec interval.
void sendPacket()
{
  myXBusServo.sendChannelDataPacket();
}


void loop() 
{
  servoValue += 10;
  myXBusServo.setServo(1, servoValue);
  delay(5);
}
