/*
 * app.c
 *
 *  Created on: 3 мар. 2020 г.
 *      Author: Rem Norton
 */
#include "app.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"
#include "proto.h"
#include "used_libs.h"

#define USB_RX_BUF_SIZE	256
#define USB_TX_BUF_SIZE	2048

#define LED_DURATION	1

uint32_t tx_off_time = 0;
uint32_t rx_off_time = 0;

static uint8_t usb_rx_buf[USB_RX_BUF_SIZE];
static uint8_t usb_tx_buf[USB_TX_BUF_SIZE];
static uint16_t usb_rx_head = 0;
static uint16_t	usb_rx_tail = 0;
static uint16_t usb_tx_idx = 0;

static CAN_USB_Mess_t	can_tx_buf[CAN_BUF_SIZE];
static uint16_t			can_tx_idx = 0;

static CAN_USB_Mess_t	can_rx_buf[CAN_BUF_SIZE];
static uint16_t			can_rx_idx = 0;

static uint8_t can_started = 0;
static uint8_t*	core_uid = (uint8_t*)UID_BASE;


static const uint16_t prescaler[CAN_BAUD_END] = {0, 400, 200, 160, 80, 40, 32, 20, 16, 10, 8, 5, 4};

//
//Private forwards
//
void start_can(uint8_t baud);
void send_via_can(CAN_USB_Mess_t* mess);
uint8_t send_via_usb(uint8_t* data, uint16_t len);
void handle_usb_tx();
void handle_usb_rx();
void handle_can_tx();
void parse_usb(CAN_USB_Header_t* hdr, uint8_t* payload);
void handle_command(CAN_USB_Header_t* hdr, uint8_t* payload);
void handle_request(CAN_USB_Header_t* hdr, uint8_t* payload);
void handle_can_rx();
void tx_led_on();
void rx_led_on();
void handle_leds();
//
//Public members
//
void app_init()
{
	HAL_CAN_Start(&hcan1);
}

FAST_RUN void app_step()
{
	handle_usb_rx();
	handle_usb_tx();
	handle_can_tx();
	handle_can_rx();
	handle_leds();

	if (hcan1.ErrorCode)
	{
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		HAL_CAN_Start(&hcan1);
	}
}

FAST_RUN void usb_rx(uint8_t* Buf, uint32_t *Len)
{
	uint16_t len1 = *Len;
	uint16_t len2 = 0;

	if (len1 >= USB_RX_BUF_SIZE)
		return;

	if ((len1 + usb_rx_head) >= USB_RX_BUF_SIZE)
	{
		len1 = USB_RX_BUF_SIZE - usb_rx_head;
		len2 = *Len - len1;
	}

	if (len1)
		memcpy(&usb_rx_buf[usb_rx_head], Buf, len1);

	if (len2)
		memcpy(usb_rx_buf, &Buf[len1], len2);

	usb_rx_head = ring_add(usb_rx_head, *Len, sizeof(usb_rx_buf));
}

//
//Private members
//

FAST_RUN void start_can(uint8_t baud)
{
	if (baud >= CAN_BAUD_END) return;
	if (baud == 0) //stop CAN
	{
		if (can_started)
		{
			HAL_CAN_Stop(&hcan1);
			can_started = 0;
			HAL_GPIO_WritePin(USB_LED, GPIO_PIN_RESET);
			HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
	else
	{
		if (can_started) HAL_CAN_Stop(&hcan1);

		HAL_CAN_DeInit(&hcan1);
		hcan1.Init.Prescaler = prescaler[baud];
		HAL_CAN_Init(&hcan1);
		HAL_CAN_Start(&hcan1);
		can_started = baud;
		HAL_GPIO_WritePin(USB_LED, GPIO_PIN_SET);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}

FAST_RUN void send_via_can(CAN_USB_Mess_t* mess)
{
	if (!can_started) return;
	if (can_tx_idx >= CAN_BUF_SIZE) return;
	memcpy(&can_tx_buf[can_tx_idx++], mess, sizeof(CAN_USB_Mess_t));
}

uint8_t send_via_usb(uint8_t* data, uint16_t len)
{
	if ((len+usb_tx_idx) < USB_TX_BUF_SIZE)
	{
		memcpy(&usb_tx_buf[usb_tx_idx], data, len);
		usb_tx_idx += len;
		return 1;
	}

	return 0;
}

FAST_RUN void handle_usb_rx()
{
	uint16_t len = ring_len(usb_rx_head, usb_rx_tail, sizeof(usb_rx_buf));
	if (len)
		usb_rx_tail = ring_seek(usb_rx_head, usb_rx_tail, _PREFIX_, usb_rx_buf, sizeof(usb_rx_buf));

	len = ring_len(usb_rx_head, usb_rx_tail, sizeof(usb_rx_buf));
	if (len < sizeof(CAN_USB_Header_t))
		return;

	CAN_USB_Header_t hdr;
	ring_extract((uint8_t*)&hdr, usb_rx_buf, usb_rx_tail, sizeof(CAN_USB_Header_t), sizeof(usb_rx_buf));
	if (len < (sizeof(hdr) + hdr.datalen))
		return;

	if (hdr.datalen > 128)
	{
		usb_rx_tail = ring_add(usb_rx_tail, 1, sizeof(usb_rx_buf));
		return;
	}

	uint8_t* payload = (uint8_t*)malloc(hdr.datalen);
	if (!payload)
		return;

	usb_rx_tail = ring_add(usb_rx_tail, sizeof(hdr), sizeof(usb_rx_buf));
	ring_extract(payload, usb_rx_buf, usb_rx_tail, hdr.datalen, sizeof(usb_rx_buf));

	parse_usb(&hdr, payload);
	usb_rx_tail = ring_add(usb_rx_tail, hdr.datalen, sizeof(usb_rx_buf));

	free(payload);
}

FAST_RUN void handle_usb_tx()
{
	if (!usb_tx_idx) return;
	if (CDC_Transmit_FS(usb_tx_buf, usb_tx_idx) == USBD_OK)
		usb_tx_idx = 0;

}

FAST_RUN void handle_can_tx()
{
	if (!can_started) return;

	if (can_tx_idx)
	{
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		{
			CAN_USB_Mess_t* mess = can_tx_buf;

			uint32_t mailbox = 0;
			CAN_TxHeaderTypeDef hdr;
			hdr.DLC = mess->flags.dlc;
			hdr.StdId = hdr.ExtId = 0;
			hdr.RTR = mess->flags.rtr?CAN_RTR_REMOTE:CAN_RTR_DATA;
			hdr.IDE = mess->flags.ide?CAN_ID_EXT:CAN_ID_STD;
			hdr.TransmitGlobalTime = 0;
			hdr.ExtId = hdr.StdId = mess->id;

			if (HAL_CAN_AddTxMessage(&hcan1, &hdr, mess->data, &mailbox) == HAL_OK)
			{
				tx_led_on();
				can_tx_idx--;
				memcpy(can_tx_buf, &can_tx_buf[1], sizeof(can_tx_buf[0])*can_tx_idx);
			}
		}
	}
}

FAST_RUN void parse_usb(CAN_USB_Header_t* hdr, uint8_t* payload)
{
	if (hdr->datalen) handle_command(hdr, payload);
	handle_request(hdr, payload);
}

FAST_RUN void handle_command(CAN_USB_Header_t* hdr, uint8_t* payload)
{
	switch(hdr->type)
	{
		case CAN_PT_MESS:
		{
			send_via_can((CAN_USB_Mess_t*)payload);
			break;
		}
		case CAN_PT_FILTER:
		{
			CAN_FilterTypeDef filter;
			CAN_USB_Filter_t* pl = (CAN_USB_Filter_t*)payload;
			filter.FilterActivation = pl->FilterActivation;
			filter.FilterBank = pl->FilterBank;
			filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
			filter.FilterIdHigh = pl->FilterIdHigh;
			filter.FilterIdLow = pl->FilterIdLow;
			filter.FilterMaskIdHigh = pl->FilterMaskIdHigh;
			filter.FilterMaskIdLow = pl->FilterMaskIdLow;
			filter.FilterMode = pl->FilterMode;
			filter.FilterScale = pl->FilterScale;
			filter.SlaveStartFilterBank = pl->SlaveStartFilterBank;

			HAL_CAN_ConfigFilter(&hcan1, &filter);
			break;
		}
		case CAN_PT_BAUD:
		{
			start_can(*payload);
			break;
		}
	}
}

FAST_RUN void handle_request(CAN_USB_Header_t* hdr, uint8_t* payload)
{
	uint8_t tx_buf[256];
	uint8_t len = 0;

	switch(hdr->type)
	{
		case CAN_PT_FILTER:
		{
			CAN_USB_Filter_t fm;
//			CAN_FilterTypeDef filter;
//			fm.FilterActivation = filter.FilterActivation;
//			fm.FilterBank = filter.FilterBank;
//			fm.FilterFIFOAssignment = filter.FilterFIFOAssignment;
//			fm.FilterIdHigh = filter.FilterIdHigh;
//			fm.FilterIdLow = filter.FilterIdLow;
//			fm.FilterMaskIdHigh = filter.FilterMaskIdHigh;
//			fm.FilterMaskIdLow = filter.FilterMaskIdLow;
//			fm.FilterMode = filter.FilterMode;
//			fm.FilterScale = filter.FilterScale;
//			fm.SlaveStartFilterBank = filter.SlaveStartFilterBank;
			memset(&fm, 0, sizeof(fm));

			len = make_usb_can_pck(CAN_PT_FILTER, &fm, sizeof(fm), tx_buf);

			break;
		}
		case CAN_PT_BAUD:
		{
			len = make_usb_can_pck(CAN_PT_BAUD, &can_started, sizeof(can_started), tx_buf);
			break;
		}
		case CAN_PT_UID:
		{
			len = make_usb_can_pck(CAN_PT_UID, core_uid, 12, tx_buf);
			break;
		}
	}

	if(len)
		send_via_usb(tx_buf, len);
}

FAST_RUN void handle_can_rx()
{
	if (can_rx_idx)
	{
		uint16_t idx = 0;
		while ((idx < can_rx_idx) && ((usb_tx_idx + sizeof(CAN_USB_Mess_t)) < USB_TX_BUF_SIZE))
		{
			rx_led_on();
			uint16_t len = make_usb_can_pck(CAN_PT_MESS, &can_rx_buf[idx], sizeof(CAN_USB_Mess_t), &usb_tx_buf[usb_tx_idx]);
			idx++;
			usb_tx_idx += len;
		}

		if (idx != can_rx_idx)
		{
			memcpy(can_rx_buf, &can_rx_buf[idx], can_rx_idx - idx);
		}
		can_rx_idx -= idx;
	}
//	uint8_t res = 1;
//	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) && res)
//	{
//		CAN_RxHeaderTypeDef hdr;
//		CAN_USB_Mess_t mess;
//		memset(mess.data, 0, sizeof(mess.data));
//		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, mess.data) != HAL_OK) return;
//
//		rx_led_on();
//		mess.id = hdr.IDE?hdr.ExtId:hdr.StdId;
//		mess.flags.ide = (hdr.IDE == CAN_ID_EXT)?1:0;
//		mess.flags.rtr = (hdr.RTR == CAN_RTR_REMOTE)?1:0;
//		mess.flags.dlc = hdr.DLC;
//		mess.filter = hdr.FilterMatchIndex;
//
//		uint8_t tx_buf[64];
//		uint8_t len = make_usb_can_pck(CAN_PT_MESS, &mess, sizeof(mess), tx_buf);
//		res = send_via_usb(tx_buf, len);
//	}
}

inline void tx_led_on()
{
	HAL_GPIO_WritePin(TX_LED, GPIO_PIN_SET);
	tx_off_time = HAL_GetTick() + LED_DURATION;
}

inline void rx_led_on()
{
	HAL_GPIO_WritePin(RX_LED, GPIO_PIN_SET);
	rx_off_time = HAL_GetTick() + LED_DURATION;
}

inline void handle_leds()
{
	if (tx_off_time && (tx_off_time < HAL_GetTick()))
	{
		HAL_GPIO_WritePin(TX_LED, GPIO_PIN_RESET);
		tx_off_time = 0;
	}

	if (rx_off_time && (rx_off_time < HAL_GetTick()))
	{
		HAL_GPIO_WritePin(RX_LED, GPIO_PIN_RESET);
		rx_off_time = 0;
	}
}


//Callback
FAST_RUN void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef hdr;
	CAN_USB_Mess_t* mess = &can_rx_buf[can_rx_idx];
	memset(mess->data, 0, sizeof(mess->data));

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, mess->data) != HAL_OK) return;

	mess->id = hdr.IDE?hdr.ExtId:hdr.StdId;
	mess->flags.ide = (hdr.IDE == CAN_ID_EXT)?1:0;
	mess->flags.rtr = (hdr.RTR == CAN_RTR_REMOTE)?1:0;
	mess->flags.dlc = hdr.DLC;
	mess->filter = hdr.FilterMatchIndex;

	if (can_rx_idx < CAN_BUF_SIZE)
		can_rx_idx++;
}
