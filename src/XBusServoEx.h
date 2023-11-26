/* XBusServoEx.h file
 * 
 * for Arduino
 *
 * Copyright (c) 2013-2016 JR PROPO
 * by Zak Sawa
 * Mod by Izumi Ninagawa
 */

#ifndef XBusServoEx_h
#define XBusServoEx_h
#include "arduino.h"

#define	kXBusInterval				14				// mSec
#define	kXbusServo900uSec			0x1249			// 900uSec
#define	kXbusServoNeutral			0x7FFF			// 1500uSec
#define	kXbusServo2100uSec			0xEDB6			// 2100uSec
#define	kXBusMaxServoNum			50				// from 1 to 50
#define	kXBusMaxServoSubID			3				// from 0 to 3
#define	kXBusServoProductIDBase		0x0200



// XBus Get/Set/Status command order
typedef enum
{
	kXBusOrder_1_Mode =				0x01,
	kXBusOrder_1_ID =				0x03,
	kXBusOrder_2_Version =			0x04,			// only for get
	kXBusOrder_2_Product =			0x05,			// only for get
	kXBusOrder_1_Unsupported =		0x06,			// only for status
	kXBusOrder_2_Reset =			0x07,			// only for set
	kXBusOrder_2_ParamWrite =		0x08,			// only for set

	kXBusOrder_2_Reverse =			0x10,
	kXBusOrder_2_Neutral =			0x11,
	kXBusOrder_2_H_Travel =			0x12,
	kXBusOrder_2_L_Travel =			0x13,
	kXBusOrder_2_H_Limit =			0x14,
	kXBusOrder_2_L_Limit =			0x15,
	kXBusOrder_1_P_Gain =			0x16,
	kXBusOrder_1_I_Gain =			0x17,
	kXBusOrder_1_D_Gain =			0x18,
	kXBusOrder_1_DeadBand =			0x19,
	kXBusOrder_2_PowerOffset =		0x1A,
	kXBusOrder_1_AlarmLevel =		0x1B,
	kXBusOrder_2_AlarmDelay =		0x1C,
	kXBusOrder_1_Angle_180 =		0x1D,
	kXBusOrder_1_SlowStart =		0x1E,	
	kXBusOrder_1_StopMode =			0x1F,	
	kXBusOrder_2_CurrentPos =		0x20,			// only for get
	kXBusOrder_1_CurrentPow =		0x21,			// only for get
	kXBusOrder_1_SpeedLimit =		0x22,
	kXBusOrder_2_MaxInteger =		0x23,	
} XBusOrder;


// XBus parameter index
typedef enum
{
	kParamIdx_Unused0 =				0x0000,
	kParamIdx_AllData_wID =			0x0001,
	kParamIdx_AllData_woID =		0x0002,
	kParamIdx_ServoID =				0x0003,
	kParamIdx_Reversed =			0x0004,
	kParamIdx_NeutralOffset =		0x0005,
	kParamIdx_TravelHigh =			0x0006,
	kParamIdx_TravelLow =			0x0007,
	kParamIdx_LimitHigh =			0x0008,
	kParamIdx_LimitLow =			0x0009,
	kParamIdx_PGainDiff =			0x000A,
	kParamIdx_IGainDiff =			0x000B,
	kParamIdx_DGainDiff =			0x000C,
	kParamIdx_DeadBandDiff =		0x000D,
	kParamIdx_PWOffsetDiff =		0x000E,
	kParamIdx_AlarmLevel =			0x000F,
	kParamIdx_AlarmDelay =			0x0010,
	kParamIdx_Angle_180 =			0x0011,
	kParamIdx_SlowStart =			0x0012,
	kParamIdx_StopMode =			0x0013,
	kParamIdx_SpeedLimit =			0x0014,
	kParamIdx_MaxIntegerDiff =		0x0015,
} XBusParamIdx;


// XBus error code
typedef enum
{
	kXBusError_NoError =			0x0000,
	kXBusError_CRCError,
	kXBusError_ServoNumOverflow,
	kXBusError_ServoNumIsZero,
	kXBusError_AddWithSameID,
	kXBusError_IDNotFound,
	kXBusError_Unsupported,
	kXBusError_OnlyForTxOnlyMode,
	kXBusError_OnlyForNormalMode,
	kXBusError_MemoryFull,
	kXBusError_TimeOut,

	kXBusError_NumOfError,
} XBusError;


// XBus servo models
#define kServo_NX8921				0x0200
#define kServo_NX3421				0x0201
#define kServo_NX588				0x0202
#define kServo_NX8925				0x0203
#define kServo_NX3425				0x0204
#define kServo_NX6421				0x0205
#define kServo_NXR89				0x0206
#define kServo_NXR34				0x0207
#define kServo_NXB8921				0x0208
#define kServo_NXB8925			 	0x0209
#define kServo_NXB89G				0x020A



class XBusServoEx
	{
		public:
    	XBusServoEx(int	dirPin, unsigned int maxServoNum);
		
		public:
			XBusError		begin(void);
			XBusError		begin1(void);
			XBusError		begin2(void);
			XBusError		begin3(void);
			XBusError		begin4(void);
			XBusError		begin5(void);

			void			end(void);
			void			end1(void);
			void			end2(void);
			void			end3(void);
			void			end4(void);
			void			end5(void);

			XBusError		addServo(char channelID, unsigned int initValue);
			XBusError		removeServo(char channelID);
			XBusError		setServo(char channelID, unsigned int value);
		
			void	sendChannelDataPacket(void);
			void	sendChannelDataPacket1(void);
			void	sendChannelDataPacket2(void);
			void	sendChannelDataPacket3(void);
			void	sendChannelDataPacket4(void);
			void	sendChannelDataPacket5(void);

			XBusError		setChannelID(char oldChannelID, char newChannelID);
			XBusError		setCommand(char channelID, char order, int* value);
			XBusError		getCommand(char channelID, char order, int* value);

			XBusError		setChannelID(char newChannelID);
			XBusError		setCommand(char order, int* value);

		
		private:
    		int				dirPinNo ;    				// pin number for XBus direction change.  if -1, no dir pin there
			int				numOfServo;						// number of servos
			unsigned int	maxServo;							// max number of servos
			uint8_t*		chPacketBuffer;				// channel data packet buffer
			uint8_t*		sendBuffer;						// serial send buffer
			char			dirty;
			volatile char	modifyServosNow;			// 

			uint8_t			crc_table(uint8_t data, uint8_t crc);
			uint8_t			crc8(uint8_t * buffer, uint8_t length);
			int				getDataSize(char	order);

			XBusError	sendCommandDataPacket(char command, char channelID, char order, int* value, char valueSize);
			XBusError	sendCommandDataPacket1(char command, char channelID, char order, int* value, char valueSize);
			XBusError	sendCommandDataPacket2(char command, char channelID, char order, int* value, char valueSize);
			XBusError	sendCommandDataPacket3(char command, char channelID, char order, int* value, char valueSize);
			XBusError	sendCommandDataPacket4(char command, char channelID, char order, int* value, char valueSize);
			XBusError	sendCommandDataPacket5(char command, char channelID, char order, int* value, char valueSize);

	};


#endif	// of XBusServoEx_h

