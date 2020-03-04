/*
 * proto.c
 *
 *  Created on: 27 но€б. 2019 г.
 *      Author: User
 */
#include "proto.h"
#include <string.h>



uint8_t make_usb_can_pck(uint8_t type, void* data, uint8_t len, uint8_t* out)
{
	if (!data || !out) return 0;
	CAN_USB_Header_t* hdr = (CAN_USB_Header_t*)out;
	hdr->prefix = _PREFIX_;
	hdr->type = type;
	hdr->datalen = len;
	memcpy(&out[sizeof(CAN_USB_Header_t)], data, len);
	crc_t* crc = (crc_t*)&out[sizeof(CAN_USB_Header_t) + len];
	*crc = calc_crc(out, sizeof(CAN_USB_Header_t) + len);

	return sizeof(CAN_USB_Header_t) + len + sizeof(crc_t);
}

crc_t calc_crc(void* data, uint8_t len)
{
	uint8_t ra, rb;
	ra = rb = 0;
	uint8_t* d = (uint8_t*)data;
	for (int i = 0; i < len; i++)
	{
		ra += d[i];
		rb += ra;
	}

	return (rb << 8) | ra;
}
