#include <XBusServoEx.h>

#define  kMaxServoNum    16       // 1 - 50
#define  kDirPinNum      -1       // Tx only mode

// This code change the channel ID from all ID to 0x02
#define  kNewChannelID   0x02

XBusServo      myXBusServo(kDirPinNum, kMaxServoNum);


void setup()
{
  myXBusServo.begin();
  myXBusServo.setChannelID(kNewChannelID);
}


void loop() 
{
}
