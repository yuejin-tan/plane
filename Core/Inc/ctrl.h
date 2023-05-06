/*
 * ctrl.h
 *
 *  Created on: 2021年5月31日
 *      Author: yuejinTan
 */

#ifndef INC_CTRL_H_
#define INC_CTRL_H_

#include "main.h"

extern uint8_t ctrl_cmd;

extern int16_t pwm_left;
extern int16_t pwm_right;
extern int16_t servo_left;
extern int16_t servo_right;

extern uint32_t NRF_buffer[3];

extern int ctrlSta;

void ctrl_refresh();

void ctrl_init();

void ctrl_decode();

#endif /* INC_CTRL_H_ */
