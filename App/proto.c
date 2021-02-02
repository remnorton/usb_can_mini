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

	return sizeof(CAN_USB_Header_t) + len;
}
