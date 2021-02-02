/*
 * board.h
 *
 *  Created on: 3 мар. 2020 г.
 *      Author: Rem Norton
 */

#ifndef BOARD_H_
#define BOARD_H_
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"

//#define FAST_RUN
#define FAST_RUN __attribute__ ((long_call, section (".code_ram")))

extern CAN_HandleTypeDef hcan1;


#define USB_DP		USB_DP_GPIO_Port, USB_DP_Pin
#define USB_LED		USB_LED_GPIO_Port, USB_LED_Pin
#define TX_LED		TX_LED_GPIO_Port, TX_LED_Pin
#define RX_LED		RX_LED_GPIO_Port, RX_LED_Pin

#endif /* BOARD_H_ */
