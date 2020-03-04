/*
 * proto.h
 *
 *  Created on: 27 ����. 2019 �.
 *      Author: User
 */

#ifndef PROTO_H_
#define PROTO_H_
#include <stdint.h>

#define _PREFIX_		0xF0


typedef	uint16_t	crc_t;

#pragma pack(1)

//! header
typedef struct
{
	uint8_t		prefix;
	uint8_t		type;
	uint8_t		datalen;
}CAN_USB_Header_t;

//! message payload
typedef struct
{
	uint32_t	id;
	uint8_t		ide;
	uint8_t		rtr;
	uint8_t		dlc;
	uint32_t    filter;
	uint8_t		data[8];
}CAN_USB_Mess_t;

//! filter payload
typedef struct
{
	uint32_t FilterIdHigh;
	uint32_t FilterIdLow;
	uint32_t FilterMaskIdHigh;
	uint32_t FilterMaskIdLow;
	uint32_t FilterFIFOAssignment;
	uint32_t FilterBank;
	uint32_t FilterMode;
	uint32_t FilterScale;
	uint32_t FilterActivation;
	uint32_t SlaveStartFilterBank;
} CAN_USB_Filter_t;

#pragma pack()

//! baud rates
enum
{
	CAN_BAUD_NONE = 0,
	CAN_BAUD_100,
	CAN_BAUD_125,
	CAN_BAUD_200,
	CAN_BAUD_250,
	CAN_BAUD_400,
	CAN_BAUD_500,
	CAN_BAUD_800,
	CAN_BAUD_1000,

	CAN_BAUD_END
};

//! packet types
enum
{
	CAN_PT_MESS = 0,
	CAN_PT_FILTER,
	CAN_PT_BAUD,
	CAN_PT_ERROR,
	CAN_PT_UID
};

//
//
//
uint8_t make_usb_can_pck(uint8_t type, void* data, uint8_t len, uint8_t* out);
crc_t calc_crc(void* data, uint8_t len);

#endif /* PROTO_H_ */