#include <I2CLiquidCrystal.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <XBusServoEx.h>


//
// defines
//
#define kMaxServoNum		16		// 1 - 50
#define kDirPinNum		2		// pin number for direction
#define kRotaryInputA		3		// pin number to input rotary phase A (INT1)
#define kRotaryInputB		4		// pin number to input rotary phase B
#define kUpButton		5		// pin number for UP button
#define kDownButton		6		// pin number for DOWN button
#define kRightButton		7		// pin number for RIGHT button
#define kLeftButton		8		// pin number for LEFT button

#define kMaxServoCommand	(sizeof(commandData) / sizeof(servoCommandRec))
#define	CurrentID		((gCurrentID & 0x3F) | ((gCurrentSubID << 6) & 0xC0))


//
// typedefs
//
typedef enum
{
	kCursor_ID,
	kCursor_SubID,
	kCursor_Function,
	kCursor_Param
} 
cursorType;

typedef struct servoCommandRec
{
char			name[5];
unsigned char		order;
unsigned char		writeIndex;
unsigned char		payloadSize;		// 5 for 2byte data / 4 for 1byte data
unsigned char		hexData;		// true if the data should be show in Hex
unsigned char		unsignedData;		// true if the data is unsigned
long			maxValue;
long			minValue;
}	servoCommandRec;
	
typedef enum
{
	kRotaly_Stop,
	kRotaly_Right,
	kRotaly_Left
} 
rotalyType;




//
// servo command data base
//
static const servoCommandRec		commandData[] = {
  //  name		order			write index				size    hex    unsin    max	min
	{"Pos ",	0x00,				0,				0,	1,	1,	0x0FFFF,  0x0000},
	{"ID  ",	0x00,				0,				0,	0,	0,	50,	  1},
	{"SbID",	0x00,				0,				0,	0,	0,	3,	  0},
	{"Ver.",	kXBusOrder_2_Version,		0,				5,	1,	1,	0,	  0},
	{"Modl",	kXBusOrder_2_Product,		0,				5,	1,	1,	0,	  0},
	{"Rev ",	kXBusOrder_2_Reverse,		kParamIdx_Reversed,		5,	0,	0,	1,	  0},
	{"Ntrl",	kXBusOrder_2_Neutral,		kParamIdx_NeutralOffset,	5,	0,	0,	300,      -300},
	{"HTrv",	kXBusOrder_2_H_Travel,		kParamIdx_TravelHigh,		5,	0,	0,	192,	  0},
	{"LTrv",	kXBusOrder_2_L_Travel,		kParamIdx_TravelLow,		5,	0,	0,	192,	  0},
	{"HLim",	kXBusOrder_2_H_Limit,		kParamIdx_LimitHigh,		5,	1,	1,	0x0FFFF,  0x0000},
	{"LLim",	kXBusOrder_2_L_Limit,		kParamIdx_LimitLow,		5,	1,	1,	0x0FFFF,  0x0000},
	{"P-Gn",	kXBusOrder_1_P_Gain,		kParamIdx_PGainDiff,		4,	0,	0,	50,	  -50},
	{"I-Gn",	kXBusOrder_1_I_Gain,		kParamIdx_IGainDiff,		4,	0,	0,	50,	  -50},
	{"D-Gn",	kXBusOrder_1_D_Gain,		kParamIdx_DGainDiff,		4,	0,	0,	50,	  -50},
	{"IMax",	kXBusOrder_2_MaxInteger,	kParamIdx_MaxIntegerDiff,	5,	0,	0,	999,	  -999},
	{"DedB",	kXBusOrder_1_DeadBand,		kParamIdx_DeadBandDiff,		4,	0,	0,	10, 	  -10},
	{"180M",	kXBusOrder_1_Angle_180,		kParamIdx_Angle_180,		4,	0,	0,	1,	  0},
	{"SpdL",	kXBusOrder_1_SpeedLimit,	kParamIdx_SpeedLimit,		4,	0,	0,	30,	  0},
	{"StpM",	kXBusOrder_1_StopMode,		kParamIdx_StopMode,		4,	0,	0,	1,	  0},
	{"POff",	kXBusOrder_2_PowerOffset,	kParamIdx_PWOffsetDiff,		5,	0,	0,	999,	  -999},
	{"SlwS",	kXBusOrder_1_SlowStart,		kParamIdx_SlowStart,		4,	0,	0,	1,	  0},
	{"AlLv",	kXBusOrder_1_AlarmLevel,	kParamIdx_AlarmLevel,		4,	0,	0,	99,	  0},
	{"AlDy",	kXBusOrder_2_AlarmDelay,	kParamIdx_AlarmDelay,		5,	0,	0,	5000,	  0},
	{"CPos",	kXBusOrder_2_CurrentPos,	0,				5,	1,	0,	0,	  0},
	{"CPow",	kXBusOrder_1_CurrentPow,	0,				4,	0,	0,	0,	  0}
	};


//
// global vars
//
XBusServo			gXBusServo(kDirPinNum, kMaxServoNum);
unsigned char			gOK2SendPacket = true;
unsigned char			gSaveCurrentValue = false;
unsigned char			gCurrentValueChanged = false;
unsigned char			gDirty = true;			// true for first update
I2CLiquidCrystal		gLCD(30, false);
unsigned int			gCurrentPos = kXbusServoNeutral;
long int			gCurrentValue = 0;
cursorType			gCurrentCursor = kCursor_ID;
volatile rotalyType		gRotalyJob = kRotaly_Stop;
unsigned char			gCurrentFunction = 0;
unsigned char			gNextFunction = 0;
unsigned char			gCurrentID = 1;
unsigned char			gCurrentSubID = 0;


//=============================================================
// setup()
//	default function for Arduino
//=============================================================
void setup()
{
	// XBus setup
	gXBusServo.begin();
	gXBusServo.addServo(0x01, kXbusServoNeutral);		 // add first servo with channelID = 0x01
	MsTimer2::set(kXBusInterval, sendPacket);
	MsTimer2::start();

	// LCD setup
	gLCD.begin(8, 2);
	gLCD.cursor();

	// Rotary encorder setup
	pinMode(kRotaryInputA, INPUT);
	pinMode(kRotaryInputB, INPUT);
	digitalWrite(kRotaryInputA, HIGH);
	digitalWrite(kRotaryInputB, HIGH);
	attachInterrupt(1, getRotary, CHANGE);

	// Buttons setup
	pinMode(kUpButton, INPUT);
	pinMode(kDownButton, INPUT);
	pinMode(kRightButton, INPUT);
	pinMode(kLeftButton, INPUT);
	digitalWrite(kUpButton, HIGH);
	digitalWrite(kDownButton, HIGH);
	digitalWrite(kRightButton, HIGH);
	digitalWrite(kLeftButton, HIGH);	
}


//=============================================================
// sendPacket()
//	This is the handler to keep to send channel command packet with 14mSec interval.
//=============================================================
void sendPacket()
{
	if (gOK2SendPacket)
		gXBusServo.sendChannelDataPacket();
}


//=============================================================
// rotaryRight()
//	This is what to do when rotary encoder turn right
//=============================================================
void rotaryRight()
{
	if (gDirty)
		return;
	
	switch (gCurrentCursor)
	{
	case kCursor_ID:
		if (gCurrentID < kXBusMaxServoNum)
		{
			gXBusServo.removeServo(CurrentID);
			gCurrentID++;
			gXBusServo.addServo(CurrentID, gCurrentPos);
			gDirty = true;
		}
		break;
		
	case kCursor_SubID:
		if (gCurrentSubID < 3)
		{
			gXBusServo.removeServo(CurrentID);
			gCurrentSubID++;
			gXBusServo.addServo(CurrentID, gCurrentPos);
			gDirty = true;
		}
		break;
		
	case kCursor_Function:
		if (gCurrentFunction < (kMaxServoCommand - 1))
		{
			gNextFunction = gCurrentFunction + 1;
			gDirty = true;
		}
		break;
		
	case kCursor_Param:
		if (commandData[gCurrentFunction].maxValue == commandData[gCurrentFunction].minValue)
			break;
		
		if (gCurrentFunction == 0)
		{
			gCurrentPos += 200;
			gDirty = true;
		}
		else if (gCurrentValue < commandData[gCurrentFunction].maxValue)
		{
			gCurrentValue++;
			gCurrentValueChanged = true;
			gDirty = true;
		}
		break;
	}
}


//=============================================================
// rotaryLeft()
//	This is what to do when rotary encoder turn left
//=============================================================
void rotaryLeft()
{
	if (gDirty)
		return;
	
	switch (gCurrentCursor)
	{
	case kCursor_ID:
		if (gCurrentID > 1)
		{
			gXBusServo.removeServo(CurrentID);		 // add first servo with channelID = 0x01
			gCurrentID--;
			gXBusServo.addServo(CurrentID, gCurrentPos);		 // add first servo with channelID = 0x01
			gDirty = true;
		}
		break;
		
	case kCursor_SubID:
		if (gCurrentSubID > 0)
		{
			gXBusServo.removeServo(CurrentID);		 // add first servo with channelID = 0x01
			gCurrentSubID--;
			gXBusServo.addServo(CurrentID, gCurrentPos);		 // add first servo with channelID = 0x01
			gDirty = true;
		}
		break;
		
	case kCursor_Function:
		if (gCurrentFunction > 0)
		{
			gNextFunction = gCurrentFunction - 1;
			gDirty = true;
		}
		break;
		
	case kCursor_Param:
		if (commandData[gCurrentFunction].maxValue == commandData[gCurrentFunction].minValue)
			break;
		
		if (gCurrentFunction == 0)
		{
			gCurrentPos -= 200;
			gDirty = true;
		}
 		else if (gCurrentValue > commandData[gCurrentFunction].minValue)
		{
			gCurrentValue--;
			gCurrentValueChanged = true;
			gDirty = true;
		}
		break;
	}
}


//=============================================================
// getRotary()
//	This is the handler to get rotary encoder
//      This code is based on the following page
//      https://sites.google.com/site/wasurenaiweb/arduino/rotaryencoder
//=============================================================
void getRotary()
{
	static unsigned char sOldRot = 0;
	
	if (! digitalRead(kRotaryInputA))
	{	 // Check to start rotating
		if (digitalRead(kRotaryInputB))
			sOldRot = 'R'; // Right turn started
		else
			sOldRot = 'L'; // Left turn started
	}
	else
	{	 // Check to stop rotating
		if (digitalRead(kRotaryInputB))
		{
			if (sOldRot == 'L') // It's still left turn
				if (gRotalyJob == kRotaly_Stop)
					gRotalyJob = kRotaly_Left;
		}
		else
		{
			if (sOldRot == 'R')	 // It's still right turn
				if (gRotalyJob == kRotaly_Stop)
					gRotalyJob = kRotaly_Right;
		}
		
		sOldRot = 0;		
	}
}


//=============================================================
// updateLCD()
//	update contents of LCD
//=============================================================
void updateLCD()
{
	String			ValueStr;
	
	// update ID
	String ID = String("00" + String(gCurrentID, DEC));
	String IDStr = String("ID" + ID.substring(ID.length() - 2) + "-" + String(gCurrentSubID, DEC));
	gLCD.setCursor(0, 0);
	gLCD.print(IDStr);
	gLCD.print("  ");
	
	// update function name
	gLCD.setCursor(0, 1);
	gLCD.print(commandData[gCurrentFunction].name);

	// update current value
	gLCD.setCursor(4, 1);
	switch(gCurrentFunction)
	{
	case 0:			// position
		ValueStr = String("0000" + String(gCurrentPos, HEX));
		break;

	default:
		if (commandData[gCurrentFunction].hexData)
			ValueStr = String("0000" + String(gCurrentValue, HEX));
		else if ((commandData[gCurrentFunction].maxValue == 1) && (commandData[gCurrentFunction].minValue == 0))
		{
			if (gCurrentValue == 1)
				ValueStr = String("  on");
			else
				ValueStr = String(" off");
		}
		else
		{
			if (gCurrentValue < 0)
				ValueStr = String("   -" + String(-gCurrentValue, DEC));
			else
				ValueStr = String("    " + String(gCurrentValue, DEC));
		}
	}
	gLCD.print(ValueStr.substring(ValueStr.length() - 4));
	
	// update cursor
	switch(gCurrentCursor)
	{
	case kCursor_ID:
		gLCD.setCursor(3, 0);
		break;

	case kCursor_SubID:
		gLCD.setCursor(5, 0);
		break;

	case kCursor_Function:
		gLCD.setCursor(3, 1);
		break;

	case kCursor_Param:
		gLCD.setCursor(7, 1);
		break;		
	}
}


//=============================================================
// buttonHandler()
//	get push button status and change flags
//=============================================================
void buttonHandler()
{
	if (! digitalRead(kUpButton))
	{
		if (gCurrentCursor == kCursor_Function)
		{
			gCurrentCursor = kCursor_ID;
			gDirty = true;
		}
		else if (gCurrentCursor == kCursor_Param)
		{
			gCurrentCursor = kCursor_SubID;
			gDirty = true;
			gSaveCurrentValue = true;
		}
	}
	else if (! digitalRead(kDownButton))
	{
		if (gCurrentCursor == kCursor_ID)
		{
			gCurrentCursor = kCursor_Function;
			gDirty = true;
		}
		else if (gCurrentCursor == kCursor_SubID)
		{
			gCurrentCursor = kCursor_Param;
			gDirty = true;
		}
	}
	else if (! digitalRead(kRightButton))
	{
		if (gCurrentCursor == kCursor_ID)
		{
			gCurrentCursor = kCursor_SubID;
			gDirty = true;
		}
		else if (gCurrentCursor == kCursor_Function)
		{
			gCurrentCursor = kCursor_Param;
			gDirty = true;
		}
	}
	else if (! digitalRead(kLeftButton))
	{
		if (gCurrentCursor == kCursor_SubID)
		{
			gCurrentCursor = kCursor_ID;
			gDirty = true;
		}
		else if (gCurrentCursor == kCursor_Param)
		{
			gCurrentCursor = kCursor_Function;
			gDirty = true;
			gSaveCurrentValue = true;
		}
	}
}


//=============================================================
// setCurrentValue()
//	set current value to the servo for temp
//=============================================================
void setCurrentValue() 
{
	int					theValue;
	XBusError		result;

	// set current value
	if (gCurrentValueChanged)
	{
		if (commandData[gCurrentFunction].writeIndex != 0)
		{
			theValue = gCurrentValue;
			for(;;)
			{
				result = gXBusServo.setCommand(CurrentID, commandData[gCurrentFunction].order, &theValue);
				delay(10);
				if (result == kXBusError_NoError)
					break;
			}
		}
	}
}


//=============================================================
// writeCurrentValue()
//	write current value to the servo
//=============================================================
void writeCurrentValue()
{
	XBusError		result;

	if (commandData[gCurrentFunction].writeIndex == 0)
	{
		// for change ID
		if ((gCurrentFunction == 1) || (gCurrentFunction == 2))
		{
			int			newChannelID;

			if (gCurrentFunction == 1)
				newChannelID = (gCurrentValue & 0x3F) | ((gCurrentSubID << 6) & 0xC0);
			else
				newChannelID = (gCurrentID & 0x3F) | ((gCurrentValue << 6) & 0xC0);

			gXBusServo.removeServo(CurrentID);

			for (;;)
			{
				result = gXBusServo.setChannelID(CurrentID, newChannelID);
				delay(10);
				if (result == kXBusError_NoError)
					break;
			}

			if (gCurrentFunction == 1)
				gCurrentID = gCurrentValue;
			else
				gCurrentSubID = gCurrentValue;
			
			gXBusServo.addServo(CurrentID, gCurrentPos);
		}
	}
	else
	{
		int			writeIndex;

		writeIndex = commandData[gCurrentFunction].writeIndex;
		for (;;)
		{
			result = gXBusServo.setCommand(CurrentID, kXBusOrder_2_ParamWrite, &writeIndex);
			delay(10);
			if (result == kXBusError_NoError)
				break;
		}
	}
}


//=============================================================
// getCurrentValue()
//	get current value from the servo
//=============================================================
void getCurrentValue()
{
	int					theValue;
	XBusError		result;

		// get current value
	switch(gCurrentFunction)
	{
	case 0:
	case 1:
	case 2:
		// do nothing
		break;

	default:
		for (;;)
		{
			result = gXBusServo.getCommand(CurrentID, commandData[gCurrentFunction].order, &theValue);
			delay(10);
			if (result == kXBusError_NoError)
				break;
		}
		gCurrentValue = theValue;
		if (commandData[gCurrentFunction].unsignedData)
			gCurrentValue &= 0x0000FFFF;
		break;
	}
}


//=============================================================
// loop()
//	default function for Arduino
//=============================================================
void loop() 
{
	buttonHandler();
	
	switch (gRotalyJob)
	{
	case kRotaly_Right:
		rotaryRight();
		break;
		
	case kRotaly_Left:
		rotaryLeft();
		break;
		
	case kRotaly_Stop:
	default:
		break;			// Do nothing
	}

	// parameter control
	if (gDirty)
	{
		gOK2SendPacket = false;
		
		setCurrentValue();

		if (gSaveCurrentValue)
			writeCurrentValue();

		if (gNextFunction != gCurrentFunction)
		{
			gCurrentFunction = gNextFunction;
			if (gCurrentFunction == 1)
				gCurrentValue = gCurrentID;
			else if (gCurrentFunction == 2)
				gCurrentValue = gCurrentSubID;
			else
				getCurrentValue();
		}
		
		updateLCD();
				
		gXBusServo.setServo(CurrentID, gCurrentPos);
		
		gOK2SendPacket = true;
	}

	gDirty = false;
	gSaveCurrentValue = false;
	gRotalyJob = kRotaly_Stop;
}



