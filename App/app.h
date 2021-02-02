/*
 * app.h
 *
 *  Created on: 3 мар. 2020 г.
 *      Author: Rem Norton
 */

#ifndef APP_H_
#define APP_H_
#include "board.h"

#define CAN_BUF_SIZE			512

void app_init();
void app_step();
void usb_rx(uint8_t* Buf, uint32_t *Len);

#endif /* APP_H_ */
