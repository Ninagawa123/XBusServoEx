/* XBusServoEx.cpp file
 * 
 * for Arduino
 *
 * Copyright (c) 2014-2016 JR PROPO
 * by Zak Sawa
 * Mod by Ninagawa Izumi
 */

#include "XBusServoEx.h"

#define	kXBusBaudrate			250000			// bps

#define	kStartOffsetOfCHData	4
#define	kCHDataSize				4
#define	kCHDataPacketCommand	0
#define	kCHDataPacketLength		1
#define	kCHDataPacketKey		2
#define	kCHDataPacketType		3

#define	kCmdDataPacketSize		8
#define	kCmdDataPacketCommand	0
#define	kCmdDataPacketLength	1
#define	kCmdDataPacketKey		2
#define	kCmdDataPacketCH_ID		3
#define	kCmdDataPacketOrder		4
#define	kCmdDataPacketData1		5
#define	kCmdDataPacketData2		6
#define	kCmdDataPacketCRC		7


// XBus Command
typedef enum
{
	kXBusCmd_Set =				0x20,
	kXBusCmd_Get =				0x21,
	kXBusCmd_Status =			0x22,
	kXBusCmd_ModeA =			0xa4
} XBusCmd; 


// XBUS device mode
typedef enum
{
	kXBusMode_Operate =			0x01,
	kXBusMode_IDSet =			0x02
} XBusMode;



//****************************************************************************
//	XBusServoEx::XBusServoEx
//		return :		none
//		parameter :	dirPin			pin number for dir change of half duplex
//					maxServoNum		max number of servo that you want to connect.
//									(limit 50)
//					this does just to reserve the buffer.  you need to 
//					add XBus servo at the beginning of your sketch
//
//		Constructor
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory allocation from here to begin()
//****************************************************************************
XBusServoEx::XBusServoEx(int dirPin, unsigned int maxServoNum)
{
	// initialize pin config
	dirPinNo = dirPin;
	if (dirPinNo >= 0)
	{
		pinMode(dirPinNo, OUTPUT);
		digitalWrite(dirPinNo, LOW);
	}

	// initialise vars
	numOfServo = 0;
	maxServo = maxServoNum;
	if (maxServo > kXBusMaxServoNum)
		maxServo = kXBusMaxServoNum;
	else if (maxServo == 0)
		maxServo = 1;
	dirty = 0;
	modifyServosNow = 0;
	chPacketBuffer = NULL;
	sendBuffer = NULL;
}


//****************************************************************************
//	XBusServoEx::begin
//		return :	error code
//		parameter :	none
//
//		This must be called before using XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory allocation from constructor to here
//					 add to check memory allocation
//****************************************************************************
XBusError XBusServoEx::begin(void)
{
	int				bufferSize;						// channel data packet buffer size

	bufferSize = kStartOffsetOfCHData + maxServo * kCHDataSize + 1;		// add 1 for CRC
	if (bufferSize < kCmdDataPacketSize)
		bufferSize = kCmdDataPacketSize;
	chPacketBuffer = (uint8_t*)malloc(bufferSize);
	if (chPacketBuffer == NULL)
		return kXBusError_MemoryFull;
	sendBuffer = (uint8_t*)malloc(bufferSize);
	if (sendBuffer == NULL)
	{
		free(chPacketBuffer);
		return kXBusError_MemoryFull;
	}

	// initialize channel packer buffer
	chPacketBuffer[kCHDataPacketCommand]	= kXBusCmd_ModeA;
	chPacketBuffer[kCHDataPacketLength]		= 0x00;
	chPacketBuffer[kCHDataPacketKey]			= 0x00;
	chPacketBuffer[kCHDataPacketType]			= 0x00;
	
	Serial.begin(kXBusBaudrate);
	Serial.setTimeout(300);

	return kXBusError_NoError;
}

//****************************************************************************
//	XBusServoEx::end
//		return :	none
//		parameter :	none
//
//		This should be called when finishing XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory free from destructor to here
//****************************************************************************
void XBusServoEx::end(void)
{
	Serial.end();
	pinMode(this->dirPinNo, INPUT);

	if (chPacketBuffer != NULL)
		free(chPacketBuffer);
	if (sendBuffer != NULL)
		free(sendBuffer);
}


//****************************************************************************
//	XBusServoEx::sendChannelDataPacket
//		return :	none
//		parameter :	none
//
//		This should be called on the timer handler like MsTimer2 when you
//		use the XBus servo.
//		2014/05/14 : add header by Sawa
//****************************************************************************
void XBusServoEx::sendChannelDataPacket(void)
{
	if (modifyServosNow)
		return;
	
	if (numOfServo > 0)
	{
		if (dirty)
		{
			memcpy(sendBuffer, chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 3);
			dirty = 0;
		}
		
		Serial.write(sendBuffer, sendBuffer[kCHDataPacketLength] + 3);
	}
}


//****************************************************************************
//	XBusServoEx::sendCommandDataPacket
//		return :		error code
//		parameter :	command		The commnad that you want to send
//					channelID	The channel ID of the XBus servo that you want to set up
//					order		The order that you want to set up
//					value		The value that you want to set / get
//					valueSize	The value size.  1 byte(char) or 2 byte(int)
//
//		This should NOT be called on the timer handler like MsTimer2 when you
//		setup the XBus servo.
//		2014/05/14 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::sendCommandDataPacket(char command, char channelID, char order, int* value, char valueSize)
{
	int					sendSize;
	int					recieveSize;
	
	// setup command
	sendBuffer[kCmdDataPacketCommand] = command;
	sendBuffer[kCmdDataPacketLength] = valueSize + 3;
	sendBuffer[kCmdDataPacketKey] = 0x00;
	sendBuffer[kCmdDataPacketCH_ID] = channelID;
	sendBuffer[kCmdDataPacketOrder] = order;
	if (valueSize == 1)						// 1 byte value
	{
		sendBuffer[kCmdDataPacketData1] = *value & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	else
	{
		sendBuffer[kCmdDataPacketData1] = (*value >> 8) & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = *value & 0x00FF;		
		sendBuffer[kCmdDataPacketCRC] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	
	// to recover channel data packet when you call sendChannelDataPacket after this
	dirty = 1;

	// send command
	sendSize = sendBuffer[kCmdDataPacketLength] + 3;
	while(Serial.read() >= 0)
		;																				// flush the receive buffer
	Serial.write(sendBuffer, sendSize);
	Serial.flush();														// wait to send all bytes

	// change bus direction to Rx mode
	if (dirPinNo >= 0)
		digitalWrite(dirPinNo, HIGH);
	
	if (channelID != 0)
	{
		// dummy read for sent packet
		recieveSize = Serial.readBytes((char*)sendBuffer, sendSize);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// read packet
		recieveSize = Serial.readBytes((char*)sendBuffer, 2);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}
		
		Serial.readBytes((char*)&(sendBuffer[kCmdDataPacketKey]), sendBuffer[kCHDataPacketLength] + 1);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);

		// check CRC
		if (crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 3) != 0)
			return kXBusError_CRCError;

		// check unsupported
		if (sendBuffer[kCmdDataPacketOrder] == kXBusOrder_1_Unsupported)
			return kXBusError_Unsupported;

		// send bcak the value
		if (valueSize == 1)						// 1 byte value
		{
			*value = sendBuffer[kCmdDataPacketData1];
			if (*value & 0x0080)
				*value |= 0xFF00;
		}
		else
		{
			*value = sendBuffer[kCmdDataPacketData1];
			*value <<= 8;
			*value |= sendBuffer[kCmdDataPacketData2];		
		}

		return kXBusError_NoError;
	}
	else
	{
		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);
		return kXBusError_NoError;
	}
}



//****************************************************************************
//	XBusServoEx::addServo
//		return :	error code
//		parameter :	channelID	channel ID of the XBus servo that you want to use
//					initValue	initial value of this XBus servo
//								use kXBusServoExNeutral for center of the XBus servo
//
//		add new servo to the buffer on this library
//		2014/05/14 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::addServo(char channelID, unsigned int initValue)
{
	int			dataOffset;

	// max check
	if (numOfServo >= maxServo)
		return kXBusError_ServoNumOverflow;
	
	// convert to servo ID
	channelID &= 0x3F;
	
	// scan servo ID that is already added
	if (numOfServo > 0)
	{
		int		servoNo;

		for (servoNo = 0; servoNo < numOfServo; servoNo++)
			if (chPacketBuffer[kStartOffsetOfCHData + kCHDataSize * servoNo] == channelID)
				return kXBusError_AddWithSameID;							// found same servo ID
	}
	
	// atomic flag on
	modifyServosNow = 1;
	
	// add new servo
	dataOffset = kStartOffsetOfCHData + kCHDataSize * numOfServo;
	numOfServo++;
	chPacketBuffer[kCHDataPacketLength] = numOfServo * kCHDataSize + 2;		// add 2 for key and type
	
	chPacketBuffer[dataOffset] = channelID;
	chPacketBuffer[dataOffset + 1] = 0x00;
	chPacketBuffer[dataOffset + 2] = (initValue >> 8) & 0x00FF;
	chPacketBuffer[dataOffset + 3] = initValue & 0x00FF;
	
	// calc CRC
	chPacketBuffer[dataOffset + 4] = crc8(chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 2);
	dirty = 1;

	// atomic flag off
	modifyServosNow = 0;
	
	return kXBusError_NoError;
}


//****************************************************************************
//	XBusServoEx::removeServo
//		return :	error code
//		parameter :	channelID	channel ID of the XBus servo that you want to remove
//
//		remove the servo from the buffer on this library
//		2014/05/14 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::removeServo(char channelID)
{
	int			dataOffset;

	// min check
	if (numOfServo == 0)
		return kXBusError_ServoNumIsZero;

	// convert to servo ID
	channelID &= 0x3F;
	
	// scan servo ID that is already added
	if (numOfServo > 0)
	{
		int		servoNo;

		for (servoNo = 0; servoNo < numOfServo; servoNo++)
			if (chPacketBuffer[kStartOffsetOfCHData + kCHDataSize * servoNo] == channelID)
			{
				// atomic flag on
				modifyServosNow = 1;
				
				// copy data after that
				if (servoNo < (numOfServo - 1))
					memcpy(&(chPacketBuffer[kStartOffsetOfCHData + kCHDataSize * servoNo]),
							&(chPacketBuffer[kStartOffsetOfCHData + kCHDataSize * (servoNo + 1)]),
							kCHDataSize * (numOfServo - servoNo - 1));
				
				// update packet size
				numOfServo--;
				chPacketBuffer[kCHDataPacketLength] = numOfServo * kCHDataSize + 2;		// add 2 for key and type

				// calc CRC
				chPacketBuffer[kStartOffsetOfCHData + kCHDataSize * numOfServo]
						= crc8(chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 2);
				dirty = 1;

				// atomic flag off
				modifyServosNow = 0;

				return kXBusError_NoError;
			}
	}
	
	return kXBusError_IDNotFound;
}


//****************************************************************************
//	XBusServoEx::setServo
//		return :		error code
//		parameter :	channelID	channel ID of the XBus servo that you want to set
//					value		value of this XBus servo
//								use kXBusServoExNeutral for center of the XBus servo
//
//		set new value to the servo
//		2014/05/14 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::setServo(char channelID, unsigned int value)
{
	int			dataOffset;

	// convert to servo ID
	channelID &= 0x3F;
	
	// scan servo ID that is already added
	if (numOfServo > 0)
	{
		int		servoNo;

		for (servoNo = 0; servoNo < numOfServo; servoNo++)
		{
			dataOffset = kStartOffsetOfCHData + kCHDataSize * servoNo;
			
			if (chPacketBuffer[dataOffset] == channelID)
			{
				// atomic flag on
				modifyServosNow = 1;
				
				// set value
				chPacketBuffer[dataOffset + 2] = (value >> 8) & 0x00FF;
				chPacketBuffer[dataOffset + 3] = value & 0x00FF;

				// calc CRC
				chPacketBuffer[kStartOffsetOfCHData + kCHDataSize * numOfServo]
						= crc8(chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 2);
				dirty = 1;

				// atomic flag off
				modifyServosNow = 0;
				
				return kXBusError_NoError;
			}
		}
	}
	
	return kXBusError_IDNotFound;
}


//****************************************************************************
//	XBusServoEx::setChannelID
//		return :	error code
//		parameter :	oldChannelID	channel IDof the XBus servo to change the ID
//					newChannelID	new channel ID for the XBus servo
//
//		set new channel ID to the XBus servo
//		2014/05/14 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::setChannelID(char oldChannelID, char newChannelID)
{
	XBusError		result;
	int					value;

	value = kXBusMode_IDSet;
	result = sendCommandDataPacket(kXBusCmd_Set, oldChannelID, kXBusOrder_1_Mode, &value, 1);
	if (result != kXBusError_NoError)
		return result;

	value = newChannelID;
	result = sendCommandDataPacket(kXBusCmd_Set, oldChannelID, kXBusOrder_1_ID, &value, 1);

	return result;
}


//****************************************************************************
//	XBusServoEx::setChannelID
//		return :	error code
//		parameter :	newChannelID	new channel ID for the XBus servo
//
//		set new channel ID to the XBus servo
//		this is only for TX only mode
//		2014/05/14 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::setChannelID(char newChannelID)
{
	XBusError		result;
	int					value;

	if (dirPinNo >= 0)
		return kXBusError_OnlyForTxOnlyMode;
	
	value = kXBusMode_IDSet;
	result = sendCommandDataPacket(kXBusCmd_Set, 0, kXBusOrder_1_Mode, &value, 1);
	if (result != kXBusError_NoError)
		return result;

	value = newChannelID;
	result = sendCommandDataPacket(kXBusCmd_Set, 0, kXBusOrder_1_ID, &value, 1);

	return result;
}


//****************************************************************************
//	XBusServoEx::getDataSize
//		return :	data size for this order
//		parameter :	order the order that you want to know
//
//		get the data size of this order
//		2014/05/15 : add header by Sawa
//****************************************************************************
int	XBusServoEx::getDataSize(char	order)
{
	char		dataSize = 1;
	
	switch(order)
	{
		case kXBusOrder_2_Version:		// only for get
		case kXBusOrder_2_Product:		// only for get
		case kXBusOrder_2_Reset:			// only for set
		case kXBusOrder_2_ParamWrite:			// only for set
		case kXBusOrder_2_Reverse:
		case kXBusOrder_2_Neutral:
		case kXBusOrder_2_H_Travel:
		case kXBusOrder_2_L_Travel:
		case kXBusOrder_2_H_Limit:
		case kXBusOrder_2_L_Limit:
		case kXBusOrder_2_PowerOffset:
		case kXBusOrder_2_AlarmDelay:
		case kXBusOrder_2_CurrentPos:		// only for get
		case kXBusOrder_2_MaxInteger:
			dataSize = 2;
	}

	return dataSize;
}


//****************************************************************************
//	XBusServoEx::setCommand
//		return :	error code
//		parameter :	channelID	channel ID of the XBus servo that you want to set to
//					order		the order that you want
//					value		the value that you want to set and return current value
//
//		send set command to the XBus servo
//		2014/05/15 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::setCommand(char channelID, char order, int* value)
{	
	return sendCommandDataPacket(kXBusCmd_Set, channelID, order, value, getDataSize(order));
}


//****************************************************************************
//	XBusServoEx::getCommand
//		return :	error code
//		parameter :	channelID	channel ID of the XBus servo that you want to get from
//					order		the order that you want
//					value		the value that you want to get from
//
//		send get command to the XBus servo
//		2014/05/15 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::getCommand(char channelID, char order, int* value)
{
	return sendCommandDataPacket(kXBusCmd_Get, channelID, order, value, getDataSize(order));
}


//****************************************************************************
//	XBusServoEx::setCommand
//		return :	error code
//		parameter :	order	the order that you want
//					value	the value that you want to set current value
//
//		send set command to the XBus servo
//		this is only for TX only mode
//		2014/05/20 : add header by Sawa
//****************************************************************************
XBusError XBusServoEx::setCommand(char order, int* value)
{
	if (dirPinNo >= 0)
		return kXBusError_OnlyForTxOnlyMode;
	
	return sendCommandDataPacket(kXBusCmd_Set, 0, order, value, getDataSize(order));
}




#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_ARCH_ESP32) || defined(__IMXRT1062__)

//****************************************************************************
//	XBusServoEx::begin1
//		return :	error code
//		parameter :	none
//
//		This must be called before using XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory allocation from constructor to here
//					 add to check memory allocation
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::begin1(void)
{
	int				bufferSize;						// channel data packet buffer size

	bufferSize = kStartOffsetOfCHData + maxServo * kCHDataSize + 1;		// add 1 for CRC
	if (bufferSize < kCmdDataPacketSize)
		bufferSize = kCmdDataPacketSize;
	chPacketBuffer = (uint8_t*)malloc(bufferSize);
	if (chPacketBuffer == NULL)
		return kXBusError_MemoryFull;
	sendBuffer = (uint8_t*)malloc(bufferSize);
	if (sendBuffer == NULL)
	{
		free(chPacketBuffer);
		return kXBusError_MemoryFull;
	}

	// initialize channel packer buffer
	chPacketBuffer[kCHDataPacketCommand]	= kXBusCmd_ModeA;
	chPacketBuffer[kCHDataPacketLength]		= 0x00;
	chPacketBuffer[kCHDataPacketKey]			= 0x00;
	chPacketBuffer[kCHDataPacketType]			= 0x00;
	
	Serial1.begin(kXBusBaudrate);
	Serial1.setTimeout(300);

	return kXBusError_NoError;
}


//****************************************************************************
//	XBusServoEx::begin2
//		return :		error code
//		parameter :	none
//
//		This must be called before using XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory allocation from constructor to here
//					 add to check memory allocation
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::begin2(void)
{
	int				bufferSize;						// channel data packet buffer size

	bufferSize = kStartOffsetOfCHData + maxServo * kCHDataSize + 1;		// add 1 for CRC
	if (bufferSize < kCmdDataPacketSize)
		bufferSize = kCmdDataPacketSize;
	chPacketBuffer = (uint8_t*)malloc(bufferSize);
	if (chPacketBuffer == NULL)
		return kXBusError_MemoryFull;
	sendBuffer = (uint8_t*)malloc(bufferSize);
	if (sendBuffer == NULL)
	{
		free(chPacketBuffer);
		return kXBusError_MemoryFull;
	}

	// initialize channel packer buffer
	chPacketBuffer[kCHDataPacketCommand]	= kXBusCmd_ModeA;
	chPacketBuffer[kCHDataPacketLength]		= 0x00;
	chPacketBuffer[kCHDataPacketKey]			= 0x00;
	chPacketBuffer[kCHDataPacketType]			= 0x00;
	
	Serial2.begin(kXBusBaudrate);
	Serial2.setTimeout(300);

	return kXBusError_NoError;
}
//****************************************************************************
//	XBusServoEx::end1
//		return :	none
//		parameter :	none
//
//		This should be called when finishing XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory free from destructor to here
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::end1(void)
{
	Serial1.end();
	pinMode(this->dirPinNo, INPUT);

	if (chPacketBuffer != NULL)
		free(chPacketBuffer);
	if (sendBuffer != NULL)
		free(sendBuffer);
}

//****************************************************************************
//	XBusServoEx::end2
//		return :	none
//		parameter :	none
//
//		This should be called when finishing XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory free from destructor to here
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::end2(void)
{
	Serial2.end();
	pinMode(this->dirPinNo, INPUT);

	if (chPacketBuffer != NULL)
		free(chPacketBuffer);
	if (sendBuffer != NULL)
		free(sendBuffer);
}


//****************************************************************************
//	XBusServoEx::sendChannelDataPacket1
//		return :	none
//		parameter :	none
//
//		This should be called on the timer handler like MsTimer2 when you
//		use the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::sendChannelDataPacket1(void)
{
	if (modifyServosNow)
		return;
	
	if (numOfServo > 0)
	{
		if (dirty)
		{
			memcpy(sendBuffer, chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 3);
			dirty = 0;
		}
		
		Serial1.write(sendBuffer, sendBuffer[kCHDataPacketLength] + 3);
	}
}

//****************************************************************************
//	XBusServoEx::sendChannelDataPacket2
//		return :	none
//		parameter :	none
//
//		This should be called on the timer handler like MsTimer2 when you
//		use the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::sendChannelDataPacket2(void)
{
	if (modifyServosNow)
		return;
	
	if (numOfServo > 0)
	{
		if (dirty)
		{
			memcpy(sendBuffer, chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 3);
			dirty = 0;
		}
		
		Serial2.write(sendBuffer, sendBuffer[kCHDataPacketLength] + 3);
	}
}


//****************************************************************************
//	XBusServoEx::sendCommandDataPacket1
//		return :	error code
//		parameter :	command		The commnad that you want to send
//					channelID	The channel ID of the XBus servo that you want to set up
//					order		The order that you want to set up
//					value		The value that you want to set / get
//					valueSize	The value size.  1 byte(char) or 2 byte(int)
//
//		This should NOT be called on the timer handler like MsTimer2 when you
//		setup the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::sendCommandDataPacket1(char command, char channelID, char order, int* value, char valueSize)
{
	int					sendSize;
	int					recieveSize;
	
	// setup command
	sendBuffer[kCmdDataPacketCommand] = command;
	sendBuffer[kCmdDataPacketLength] = valueSize + 3;
	sendBuffer[kCmdDataPacketKey] = 0x00;
	sendBuffer[kCmdDataPacketCH_ID] = channelID;
	sendBuffer[kCmdDataPacketOrder] = order;
	if (valueSize == 1)						// 1 byte value
	{
		sendBuffer[kCmdDataPacketData1] = *value & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	else
	{
		sendBuffer[kCmdDataPacketData1] = (*value >> 8) & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = *value & 0x00FF;		
		sendBuffer[kCmdDataPacketCRC] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	
	// to recover channel data packet when you call sendChannelDataPacket after this
	dirty = 1;

	// send command
	sendSize = sendBuffer[kCmdDataPacketLength] + 3;
	while(Serial1.read() >= 0)
		;																				// flush the receive buffer
	Serial1.write(sendBuffer, sendSize);
	Serial1.flush();														// wait to send all bytes

	// change bus direction to Rx mode
	if (dirPinNo >= 0)
		digitalWrite(dirPinNo, HIGH);
	
	if (channelID != 0)
	{
		// dummy read for sent packet
		recieveSize = Serial1.readBytes((char*)sendBuffer, sendSize);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// read packet
		recieveSize = Serial1.readBytes((char*)sendBuffer, 2);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}
		
		Serial1.readBytes((char*)&(sendBuffer[kCmdDataPacketKey]), sendBuffer[kCHDataPacketLength] + 1);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);

		// check CRC
		if (crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 3) != 0)
			return kXBusError_CRCError;

		// check unsupported
		if (sendBuffer[kCmdDataPacketOrder] == kXBusOrder_1_Unsupported)
			return kXBusError_Unsupported;

		// send bcak the value
		if (valueSize == 1)						// 1 byte value
		{
			*value = sendBuffer[kCmdDataPacketData1];
			if (*value & 0x0080)
				*value |= 0xFF00;
		}
		else
		{
			*value = sendBuffer[kCmdDataPacketData1];
			*value <<= 8;
			*value |= sendBuffer[kCmdDataPacketData2];		
		}

		return kXBusError_NoError;
	}
	else
	{
		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);
		return kXBusError_NoError;
	}
}

//****************************************************************************
//	XBusServoEx::sendCommandDataPacket2
//		return :	error code
//		parameter :	command		The commnad that you want to send
//					channelID	The channel ID of the XBus servo that you want to set up
//					order		The order that you want to set up
//					value		The value that you want to set / get
//					valueSize	The value size.  1 byte(char) or 2 byte(int)
//
//		This should NOT be called on the timer handler like MsTimer2 when you
//		setup the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::sendCommandDataPacket2(char command, char channelID, char order, int* value, char valueSize)
{
	int					sendSize;
	int					recieveSize;
	
	// setup command
	sendBuffer[kCmdDataPacketCommand] = command;
	sendBuffer[kCmdDataPacketLength] = valueSize + 3;
	sendBuffer[kCmdDataPacketKey] = 0x00;
	sendBuffer[kCmdDataPacketCH_ID] = channelID;
	sendBuffer[kCmdDataPacketOrder] = order;
	if (valueSize == 1)						// 1 byte value
	{
		sendBuffer[kCmdDataPacketData1] = *value & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	else
	{
		sendBuffer[kCmdDataPacketData1] = (*value >> 8) & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = *value & 0x00FF;		
		sendBuffer[kCmdDataPacketCRC] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	
	// to recover channel data packet when you call sendChannelDataPacket after this
	dirty = 1;

	// send command
	sendSize = sendBuffer[kCmdDataPacketLength] + 3;
	while(Serial2.read() >= 0)
		;																				// flush the receive buffer
	Serial2.write(sendBuffer, sendSize);
	Serial2.flush();														// wait to send all bytes

	// change bus direction to Rx mode
	if (dirPinNo >= 0)
		digitalWrite(dirPinNo, HIGH);
	
	if (channelID != 0)
	{
		// dummy read for sent packet
		recieveSize = Serial2.readBytes((char*)sendBuffer, sendSize);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// read packet
		recieveSize = Serial2.readBytes((char*)sendBuffer, 2);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}
		
		Serial2.readBytes((char*)&(sendBuffer[kCmdDataPacketKey]), sendBuffer[kCHDataPacketLength] + 1);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);

		// check CRC
		if (crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 3) != 0)
			return kXBusError_CRCError;

		// check unsupported
		if (sendBuffer[kCmdDataPacketOrder] == kXBusOrder_1_Unsupported)
			return kXBusError_Unsupported;

		// send bcak the value
		if (valueSize == 1)						// 1 byte value
		{
			*value = sendBuffer[kCmdDataPacketData1];
			if (*value & 0x0080)
				*value |= 0xFF00;
		}
		else
		{
			*value = sendBuffer[kCmdDataPacketData1];
			*value <<= 8;
			*value |= sendBuffer[kCmdDataPacketData2];		
		}

		return kXBusError_NoError;
	}
	else
	{
		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);
		return kXBusError_NoError;
	}
}

#endif



#if defined(ARDUINO_AVR_MEGA2560) || defined(__IMXRT1062__)

//****************************************************************************
//	XBusServoEx::begin3
//		return :	error code
//		parameter :	none
//
//		This must be called before using XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory allocation from constructor to here
//					 add to check memory allocation
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::begin3(void)
{
	int				bufferSize;						// channel data packet buffer size

	bufferSize = kStartOffsetOfCHData + maxServo * kCHDataSize + 1;		// add 1 for CRC
	if (bufferSize < kCmdDataPacketSize)
		bufferSize = kCmdDataPacketSize;
	chPacketBuffer = (uint8_t*)malloc(bufferSize);
	if (chPacketBuffer == NULL)
		return kXBusError_MemoryFull;
	sendBuffer = (uint8_t*)malloc(bufferSize);
	if (sendBuffer == NULL)
	{
		free(chPacketBuffer);
		return kXBusError_MemoryFull;
	}

	// initialize channel packer buffer
	chPacketBuffer[kCHDataPacketCommand]	= kXBusCmd_ModeA;
	chPacketBuffer[kCHDataPacketLength]		= 0x00;
	chPacketBuffer[kCHDataPacketKey]			= 0x00;
	chPacketBuffer[kCHDataPacketType]			= 0x00;
	
	Serial3.begin(kXBusBaudrate);
	Serial3.setTimeout(300);

	return kXBusError_NoError;
}
//****************************************************************************
//	XBusServoEx::end3
//		return :	none
//		parameter :	none
//
//		This should be called when finishing XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory free from destructor to here
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::end3(void)
{
	Serial3.end();
	pinMode(this->dirPinNo, INPUT);

	if (chPacketBuffer != NULL)
		free(chPacketBuffer);
	if (sendBuffer != NULL)
		free(sendBuffer);
}


//****************************************************************************
//	XBusServoEx::sendChannelDataPacket3
//		return :	none
//		parameter :	none
//
//		This should be called on the timer handler like MsTimer2 when you
//		use the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::sendChannelDataPacket3(void)
{
	if (modifyServosNow)
		return;
	
	if (numOfServo > 0)
	{
		if (dirty)
		{
			memcpy(sendBuffer, chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 3);
			dirty = 0;
		}
		
		Serial3.write(sendBuffer, sendBuffer[kCHDataPacketLength] + 3);
	}
}

//****************************************************************************
//	XBusServoEx::sendCommandDataPacket3
//		return :	error code
//		parameter :	command		The commnad that you want to send
//					channelID	The channel ID of the XBus servo that you want to set up
//					order		The order that you want to set up
//					value		The value that you want to set / get
//					valueSize	The value size.  1 byte(char) or 2 byte(int)
//
//		This should NOT be called on the timer handler like MsTimer2 when you
//		setup the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::sendCommandDataPacket3(char command, char channelID, char order, int* value, char valueSize)
{
	int					sendSize;
	int					recieveSize;
	
	// setup command
	sendBuffer[kCmdDataPacketCommand] = command;
	sendBuffer[kCmdDataPacketLength] = valueSize + 3;
	sendBuffer[kCmdDataPacketKey] = 0x00;
	sendBuffer[kCmdDataPacketCH_ID] = channelID;
	sendBuffer[kCmdDataPacketOrder] = order;
	if (valueSize == 1)						// 1 byte value
	{
		sendBuffer[kCmdDataPacketData1] = *value & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	else
	{
		sendBuffer[kCmdDataPacketData1] = (*value >> 8) & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = *value & 0x00FF;		
		sendBuffer[kCmdDataPacketCRC] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	
	// to recover channel data packet when you call sendChannelDataPacket after this
	dirty = 1;

	// send command
	sendSize = sendBuffer[kCmdDataPacketLength] + 3;
	while(Serial3.read() >= 0)
		;																				// flush the receive buffer
	Serial3.write(sendBuffer, sendSize);
	Serial3.flush();														// wait to send all bytes

	// change bus direction to Rx mode
	if (dirPinNo >= 0)
		digitalWrite(dirPinNo, HIGH);
	
	if (channelID != 0)
	{
		// dummy read for sent packet
		recieveSize = Serial3.readBytes((char*)sendBuffer, sendSize);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// read packet
		recieveSize = Serial3.readBytes((char*)sendBuffer, 2);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}
		
		Serial3.readBytes((char*)&(sendBuffer[kCmdDataPacketKey]), sendBuffer[kCHDataPacketLength] + 1);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);

		// check CRC
		if (crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 3) != 0)
			return kXBusError_CRCError;

		// check unsupported
		if (sendBuffer[kCmdDataPacketOrder] == kXBusOrder_1_Unsupported)
			return kXBusError_Unsupported;

		// send bcak the value
		if (valueSize == 1)						// 1 byte value
		{
			*value = sendBuffer[kCmdDataPacketData1];
			if (*value & 0x0080)
				*value |= 0xFF00;
		}
		else
		{
			*value = sendBuffer[kCmdDataPacketData1];
			*value <<= 8;
			*value |= sendBuffer[kCmdDataPacketData2];		
		}

		return kXBusError_NoError;
	}
	else
	{
		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);
		return kXBusError_NoError;
	}
}
#endif



#if defined(__IMXRT1062__)

//****************************************************************************
//	XBusServoEx::begin4
//		return :	error code
//		parameter :	none
//
//		This must be called before using XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory allocation from constructor to here
//					 add to check memory allocation
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::begin4(void)
{
	int				bufferSize;						// channel data packet buffer size

	bufferSize = kStartOffsetOfCHData + maxServo * kCHDataSize + 1;		// add 1 for CRC
	if (bufferSize < kCmdDataPacketSize)
		bufferSize = kCmdDataPacketSize;
	chPacketBuffer = (uint8_t*)malloc(bufferSize);
	if (chPacketBuffer == NULL)
		return kXBusError_MemoryFull;
	sendBuffer = (uint8_t*)malloc(bufferSize);
	if (sendBuffer == NULL)
	{
		free(chPacketBuffer);
		return kXBusError_MemoryFull;
	}

	// initialize channel packer buffer
	chPacketBuffer[kCHDataPacketCommand]	= kXBusCmd_ModeA;
	chPacketBuffer[kCHDataPacketLength]		= 0x00;
	chPacketBuffer[kCHDataPacketKey]			= 0x00;
	chPacketBuffer[kCHDataPacketType]			= 0x00;
	
	Serial4.begin(kXBusBaudrate);
	Serial4.setTimeout(300);

	return kXBusError_NoError;
}


//****************************************************************************
//	XBusServoEx::begin5
//		return :	error code
//		parameter :	none
//
//		This must be called before using XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory allocation from constructor to here
//					 add to check memory allocation
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::begin5(void)
{
	int				bufferSize;						// channel data packet buffer size

	bufferSize = kStartOffsetOfCHData + maxServo * kCHDataSize + 1;		// add 1 for CRC
	if (bufferSize < kCmdDataPacketSize)
		bufferSize = kCmdDataPacketSize;
	chPacketBuffer = (uint8_t*)malloc(bufferSize);
	if (chPacketBuffer == NULL)
		return kXBusError_MemoryFull;
	sendBuffer = (uint8_t*)malloc(bufferSize);
	if (sendBuffer == NULL)
	{
		free(chPacketBuffer);
		return kXBusError_MemoryFull;
	}

	// initialize channel packer buffer
	chPacketBuffer[kCHDataPacketCommand]	= kXBusCmd_ModeA;
	chPacketBuffer[kCHDataPacketLength]		= 0x00;
	chPacketBuffer[kCHDataPacketKey]			= 0x00;
	chPacketBuffer[kCHDataPacketType]			= 0x00;
	
	Serial5.begin(kXBusBaudrate);
	Serial5.setTimeout(300);

	return kXBusError_NoError;
}


//****************************************************************************
//	XBusServoEx::end4
//		return :	none
//		parameter :	none
//
//		This should be called when finishing XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory free from destructor to here
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::end4(void)
{
	Serial4.end();
	pinMode(this->dirPinNo, INPUT);

	if (chPacketBuffer != NULL)
		free(chPacketBuffer);
	if (sendBuffer != NULL)
		free(sendBuffer);
}

//****************************************************************************
//	XBusServoEx::end5
//		return :	none
//		parameter :	none
//
//		This should be called when finishing XBus.
//		2014/05/14 : add header by Sawa
//		2014/10/09 : move memory free from destructor to here
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::end5(void)
{
	Serial5.end();
	pinMode(this->dirPinNo, INPUT);

	if (chPacketBuffer != NULL)
		free(chPacketBuffer);
	if (sendBuffer != NULL)
		free(sendBuffer);
}

//****************************************************************************
//	XBusServoEx::sendChannelDataPacket4
//		return :	none
//		parameter :	none
//
//		This should be called on the timer handler like MsTimer2 when you
//		use the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::sendChannelDataPacket4(void)
{
	if (modifyServosNow)
		return;
	
	if (numOfServo > 0)
	{
		if (dirty)
		{
			memcpy(sendBuffer, chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 3);
			dirty = 0;
		}
		
		Serial4.write(sendBuffer, sendBuffer[kCHDataPacketLength] + 3);
	}
}

//****************************************************************************
//	XBusServoEx::sendChannelDataPacket5
//		return :	none
//		parameter :	none
//
//		This should be called on the timer handler like MsTimer2 when you
//		use the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
void XBusServoEx::sendChannelDataPacket5(void)
{
	if (modifyServosNow)
		return;
	
	if (numOfServo > 0)
	{
		if (dirty)
		{
			memcpy(sendBuffer, chPacketBuffer, chPacketBuffer[kCHDataPacketLength] + 3);
			dirty = 0;
		}
		
		Serial5.write(sendBuffer, sendBuffer[kCHDataPacketLength] + 3);
	}
}

//****************************************************************************
//	XBusServoEx::sendCommandDataPacket4
//		return :	error code
//		parameter :	command		The commnad that you want to send
//					channelID	The channel ID of the XBus servo that you want to set up
//					order		The order that you want to set up
//					value		The value that you want to set / get
//					valueSize	The value size.  1 byte(char) or 2 byte(int)
//
//		This should NOT be called on the timer handler like MsTimer2 when you
//		setup the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::sendCommandDataPacket4(char command, char channelID, char order, int* value, char valueSize)
{
	int					sendSize;
	int					recieveSize;
	
	// setup command
	sendBuffer[kCmdDataPacketCommand] = command;
	sendBuffer[kCmdDataPacketLength] = valueSize + 3;
	sendBuffer[kCmdDataPacketKey] = 0x00;
	sendBuffer[kCmdDataPacketCH_ID] = channelID;
	sendBuffer[kCmdDataPacketOrder] = order;
	if (valueSize == 1)						// 1 byte value
	{
		sendBuffer[kCmdDataPacketData1] = *value & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	else
	{
		sendBuffer[kCmdDataPacketData1] = (*value >> 8) & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = *value & 0x00FF;		
		sendBuffer[kCmdDataPacketCRC] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	
	// to recover channel data packet when you call sendChannelDataPacket after this
	dirty = 1;

	// send command
	sendSize = sendBuffer[kCmdDataPacketLength] + 3;
	while(Serial4.read() >= 0)
		;																				// flush the receive buffer
	Serial4.write(sendBuffer, sendSize);
	Serial4.flush();														// wait to send all bytes

	// change bus direction to Rx mode
	if (dirPinNo >= 0)
		digitalWrite(dirPinNo, HIGH);
	
	if (channelID != 0)
	{
		// dummy read for sent packet
		recieveSize = Serial4.readBytes((char*)sendBuffer, sendSize);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// read packet
		recieveSize = Serial4.readBytes((char*)sendBuffer, 2);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}
		
		Serial4.readBytes((char*)&(sendBuffer[kCmdDataPacketKey]), sendBuffer[kCHDataPacketLength] + 1);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);

		// check CRC
		if (crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 3) != 0)
			return kXBusError_CRCError;

		// check unsupported
		if (sendBuffer[kCmdDataPacketOrder] == kXBusOrder_1_Unsupported)
			return kXBusError_Unsupported;

		// send bcak the value
		if (valueSize == 1)						// 1 byte value
		{
			*value = sendBuffer[kCmdDataPacketData1];
			if (*value & 0x0080)
				*value |= 0xFF00;
		}
		else
		{
			*value = sendBuffer[kCmdDataPacketData1];
			*value <<= 8;
			*value |= sendBuffer[kCmdDataPacketData2];		
		}

		return kXBusError_NoError;
	}
	else
	{
		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);
		return kXBusError_NoError;
	}
}


//****************************************************************************
//	XBusServoEx::sendCommandDataPacket5
//		return :	error code
//		parameter :	command		The commnad that you want to send
//					channelID	The channel ID of the XBus servo that you want to set up
//					order		The order that you want to set up
//					value		The value that you want to set / get
//					valueSize	The value size.  1 byte(char) or 2 byte(int)
//
//		This should NOT be called on the timer handler like MsTimer2 when you
//		setup the XBus servo.
//		2014/05/14 : add header by Sawa
// 		2023/11/26 : added by Izumi Ninagawa
//****************************************************************************
XBusError XBusServoEx::sendCommandDataPacket5(char command, char channelID, char order, int* value, char valueSize)
{
	int					sendSize;
	int					recieveSize;
	
	// setup command
	sendBuffer[kCmdDataPacketCommand] = command;
	sendBuffer[kCmdDataPacketLength] = valueSize + 3;
	sendBuffer[kCmdDataPacketKey] = 0x00;
	sendBuffer[kCmdDataPacketCH_ID] = channelID;
	sendBuffer[kCmdDataPacketOrder] = order;
	if (valueSize == 1)						// 1 byte value
	{
		sendBuffer[kCmdDataPacketData1] = *value & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	else
	{
		sendBuffer[kCmdDataPacketData1] = (*value >> 8) & 0x00FF;
		sendBuffer[kCmdDataPacketData2] = *value & 0x00FF;		
		sendBuffer[kCmdDataPacketCRC] = crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 2);
	}
	
	// to recover channel data packet when you call sendChannelDataPacket after this
	dirty = 1;

	// send command
	sendSize = sendBuffer[kCmdDataPacketLength] + 3;
	while(Serial5.read() >= 0)
		;																				// flush the receive buffer
	Serial5.write(sendBuffer, sendSize);
	Serial5.flush();														// wait to send all bytes

	// change bus direction to Rx mode
	if (dirPinNo >= 0)
		digitalWrite(dirPinNo, HIGH);
	
	if (channelID != 0)
	{
		// dummy read for sent packet
		recieveSize = Serial5.readBytes((char*)sendBuffer, sendSize);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// read packet
		recieveSize = Serial5.readBytes((char*)sendBuffer, 2);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}
		
		Serial5.readBytes((char*)&(sendBuffer[kCmdDataPacketKey]), sendBuffer[kCHDataPacketLength] + 1);
		if (recieveSize == 0)
		{
		// change bus direction to Tx mode
			if (dirPinNo >= 0)
				digitalWrite(dirPinNo, LOW);
			return kXBusError_TimeOut;
		}

		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);

		// check CRC
		if (crc8(sendBuffer, sendBuffer[kCHDataPacketLength] + 3) != 0)
			return kXBusError_CRCError;

		// check unsupported
		if (sendBuffer[kCmdDataPacketOrder] == kXBusOrder_1_Unsupported)
			return kXBusError_Unsupported;

		// send bcak the value
		if (valueSize == 1)						// 1 byte value
		{
			*value = sendBuffer[kCmdDataPacketData1];
			if (*value & 0x0080)
				*value |= 0xFF00;
		}
		else
		{
			*value = sendBuffer[kCmdDataPacketData1];
			*value <<= 8;
			*value |= sendBuffer[kCmdDataPacketData2];		
		}

		return kXBusError_NoError;
	}
	else
	{
		// change bus direction to Tx mode
		if (dirPinNo >= 0)
			digitalWrite(dirPinNo, LOW);
		return kXBusError_NoError;
	}
}
#endif



//****************************************************************************
// for CRC
static uint8_t s_crc_array[256] =
	{
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
		0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
		0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
		0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
		0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
		0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
		0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
		0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
		0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
		0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
		0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
		0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
		0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
		0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
		0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
		0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
		0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
		0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
		0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
		0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
		0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
		0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
		0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
		0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
		0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
		0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
		0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
		0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
		0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
		0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
		0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
	};


uint8_t XBusServoEx::crc_table(uint8_t  data, uint8_t  crc)
	{
		uint16_t  index = (data ^ crc) & 0xff;
		
		crc = s_crc_array[index];
		return crc;
	}


uint8_t XBusServoEx::crc8(uint8_t* buffer, uint8_t  length)
	{
		uint8_t  crc = 0;
		
		while (length-- > 0)
			crc = crc_table(*buffer++, crc);
		return crc;
	}




