# XBusServoEx
This library allows you to use XBusServoLib on Serial for Arduino, Serial1&2 for ESP32 and MEGA, Serial 3to5 for Teensy.

# Manual
https://onedrive.live.com/?authkey=%21AAMbJRFSolJPzEY&id=1544AD96D75081BB%2137929&cid=1544AD96D75081BB
- XBusServoLibrary.pdf
- XbusServoSpec.pdfz

# How to use Serial1 to 5
If you are using Serial2, please use begin2(); instead of begin();.
When using Serial2, the following four commands should be modified:
- begin(); -> begin2();
- end(); -> end2();
- sendChannelDataPacket(); -> sendChannelDataPacket2();
- sendCommandDataPacket(); -> sendCommandDataPacket2();

The same applies when using other serial interfaces.

# DEMO
To use ESP32DevKitC and Serial2, try sample sketch ESP32_demo.ino.