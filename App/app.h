/*
 * app.h
 *
 *  Created on: 3 мар. 2020 г.
 *      Author: Rem Norton
 */

#ifndef APP_H_
#define APP_H_
#include "board.h"

extern uint32_t tx_off_time;
extern uint32_t rx_off_time;
extern uint32_t usb_off_time;

void app_init();
void app_step();
void usb_rx(uint8_t* Buf, uint32_t *Len);

#endif /* APP_H_ */
