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

#define USB_RX_BUF_SIZE	128
#define USB_TX_BUF_SIZE	512
#define LED_DURATION	5

uint32_t tx_off_time = 0;
uint32_t rx_off_time = 0;
uint32_t usb_off_time = 0;
uint32_t usb_tx_time = 0;

uint16_t usb_rx_idx = 0;
uint16_t usb_tx_idx = 0;

static uint8_t usb_rx_buf[USB_RX_BUF_SIZE];
static uint8_t usb_tx_buf[USB_TX_BUF_SIZE];
static uint8_t can_started = 0;
static uint8_t*	core_uid = (uint8_t*)UID_BASE;

static const uint8_t prescaler[CAN_BAUD_END] = {0, 40, 32, 20, 16, 10, 8, 5, 4};

//
//Private forwards
//
void start_can(uint8_t baud);
void send_via_can(CAN_USB_Mess_t* mess);
void send_via_usb(uint8_t* data, uint16_t len);
void handle_usb_tx();
void handle_usb_rx();
void parse_usb(CAN_USB_Header_t* hdr, uint8_t* payload);
void handle_command(CAN_USB_Header_t* hdr, uint8_t* payload);
void handle_request(CAN_USB_Header_t* hdr, uint8_t* payload);
void handle_can();
void usb_led_on();
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

void app_step()
{
	handle_usb_rx();
	handle_usb_tx();
	handle_can();
	handle_leds();
}

void usb_rx(uint8_t* Buf, uint32_t *Len)
{
	if ((*Len + usb_rx_idx) < USB_RX_BUF_SIZE)
	{
		memcpy(&usb_rx_buf[usb_rx_idx], Buf, *Len);
		usb_rx_idx += *Len;
		usb_led_on();
	}
}

//
//Private members
//

void start_can(uint8_t baud)
{
	if (baud >= CAN_BAUD_END) return;
	if ((baud == 0) && can_started) //stop CAN
	{
		HAL_CAN_Stop(&hcan1);
		can_started = 0;
	}
	else
	{
		if (can_started) HAL_CAN_Stop(&hcan1);

		HAL_CAN_DeInit(&hcan1);
		hcan1.Init.Prescaler = prescaler[baud];
		HAL_CAN_Init(&hcan1);
		HAL_CAN_Start(&hcan1);
		can_started = baud;
	}
}

void send_via_can(CAN_USB_Mess_t* mess)
{
	if (!can_started) return;
	uint32_t mailbox = 0;
	CAN_TxHeaderTypeDef hdr;
	hdr.DLC = mess->dlc;
	hdr.StdId = hdr.ExtId = 0;
	hdr.RTR = mess->rtr?CAN_RTR_REMOTE:CAN_RTR_DATA;;
	hdr.IDE = mess->ide?CAN_ID_EXT:CAN_ID_STD;;
	hdr.TransmitGlobalTime = 0;
	if (mess->ide) hdr.ExtId = mess->id;
	else hdr.StdId = mess->id;
	HAL_CAN_AddTxMessage(&hcan1, &hdr, mess->data, &mailbox);
	tx_led_on();
}

void send_via_usb(uint8_t* data, uint16_t len)
{
	if ((len+usb_tx_idx) < USB_RX_BUF_SIZE)
	{
		memcpy(&usb_tx_buf[usb_tx_idx], data, len);
		usb_tx_idx += len;
	}
}

void handle_usb_rx()
{
	uint16_t offs = 0;
	while ((usb_rx_buf[offs] != _PREFIX_) && (offs < usb_rx_idx)) offs++;

	if (usb_rx_idx < (sizeof(CAN_USB_Header_t) + sizeof(crc_t)+offs)) return;

	CAN_USB_Header_t* hdr = (CAN_USB_Header_t*)&usb_rx_buf[offs];
	if (usb_rx_idx < (sizeof(CAN_USB_Header_t) + hdr->datalen + sizeof(crc_t)+offs)) //incomplette packet
		return;

	crc_t* pcrc = (crc_t*)&usb_rx_buf[sizeof(CAN_USB_Header_t) + hdr->datalen + offs];
	crc_t  crc = calc_crc(&usb_rx_buf[offs], sizeof(CAN_USB_Header_t) + hdr->datalen);

	if (*pcrc != crc) offs += 1;
	else
	{
		parse_usb(hdr, &usb_rx_buf[sizeof(CAN_USB_Header_t)]);
		offs += sizeof(CAN_USB_Header_t) + hdr->datalen + sizeof(crc_t);
	}

	memcpy(usb_rx_buf, &usb_rx_buf[offs], USB_RX_BUF_SIZE - offs);
	usb_rx_idx -= offs;
}

void handle_usb_tx()
{
	if (!usb_tx_idx) return;
	if (usb_tx_time > HAL_GetTick()) return;
	if (CDC_Transmit_FS(usb_tx_buf, usb_tx_idx) == USBD_OK)
	{
		usb_tx_idx = 0;
		usb_tx_time = 0;
		usb_led_on();
	}
	else usb_tx_time = HAL_GetTick() + 5;
}

void parse_usb(CAN_USB_Header_t* hdr, uint8_t* payload)
{
	if (hdr->datalen) handle_command(hdr, payload);
	handle_request(hdr, payload);
}

void handle_command(CAN_USB_Header_t* hdr, uint8_t* payload)
{
	switch(hdr->type)
	{
		case CAN_PT_MESS:
		{
			CAN_USB_Mess_t* pl = (CAN_USB_Mess_t*)payload;
			send_via_can(pl);
			break;
		}
		case CAN_PT_FILTER:
		{
			CAN_FilterTypeDef filter;
			CAN_USB_Filter_t* pl = (CAN_USB_Filter_t*)payload;
			filter.FilterActivation = pl->FilterActivation;
			filter.FilterBank = pl->FilterBank;
			filter.FilterFIFOAssignment = pl->FilterFIFOAssignment;
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

void handle_request(CAN_USB_Header_t* hdr, uint8_t* payload)
{
	uint8_t tx_buf[256];
	uint8_t len = 0;

	switch(hdr->type)
	{
		case CAN_PT_FILTER:
		{
			CAN_USB_Filter_t fm;
			CAN_FilterTypeDef filter;
			fm.FilterActivation = filter.FilterActivation;
			fm.FilterBank = filter.FilterBank;
			fm.FilterFIFOAssignment = filter.FilterFIFOAssignment;
			fm.FilterIdHigh = filter.FilterIdHigh;
			fm.FilterIdLow = filter.FilterIdLow;
			fm.FilterMaskIdHigh = filter.FilterMaskIdHigh;
			fm.FilterMaskIdLow = filter.FilterMaskIdLow;
			fm.FilterMode = filter.FilterMode;
			fm.FilterScale = filter.FilterScale;
			fm.SlaveStartFilterBank = filter.SlaveStartFilterBank;

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

	if(len) send_via_usb(tx_buf, len);
}

void handle_can()
{
	while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
	{
		CAN_RxHeaderTypeDef hdr;
		CAN_USB_Mess_t mess;
		memset(mess.data, 0, sizeof(mess.data));
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, mess.data) != HAL_OK) return;

		rx_led_on();
		mess.id = hdr.IDE?hdr.ExtId:hdr.StdId;
		mess.ide = hdr.IDE;
		mess.rtr = hdr.RTR;
		mess.dlc = hdr.DLC;
		mess.filter = hdr.FilterMatchIndex;

		uint8_t tx_buf[256];
		uint8_t len = make_usb_can_pck(CAN_PT_MESS, &mess, sizeof(mess), tx_buf);
		send_via_usb(tx_buf, len);
	}
}

void usb_led_on()
{
	HAL_GPIO_WritePin(USB_LED, GPIO_PIN_SET);
	usb_off_time = HAL_GetTick() + LED_DURATION;
}

void tx_led_on()
{
	HAL_GPIO_WritePin(TX_LED, GPIO_PIN_SET);
	tx_off_time = HAL_GetTick() + LED_DURATION;
}

void rx_led_on()
{
	HAL_GPIO_WritePin(RX_LED, GPIO_PIN_SET);
	rx_off_time = HAL_GetTick() + LED_DURATION;
}

void handle_leds()
{
	if (usb_off_time && (usb_off_time < HAL_GetTick()))
	{
		HAL_GPIO_WritePin(USB_LED, GPIO_PIN_RESET);
		usb_off_time = 0;
	}

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

