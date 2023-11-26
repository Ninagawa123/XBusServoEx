#include <XBusServoEx.h>

#define  kMaxServoNum    16       // 1 - 50
#define  kDirPinNum      2        // pin number for direction

// This code change the channel ID from 0x01 to 0x02
#define  kOldChannelID   0x01
#define  kNewChannelID   0x02

XBusServo      myXBusServo(kDirPinNum, kMaxServoNum);


void setup()
{
  myXBusServo.begin();
  myXBusServo.setChannelID(kOldChannelID, kNewChannelID);
}


void loop() 
{
}
