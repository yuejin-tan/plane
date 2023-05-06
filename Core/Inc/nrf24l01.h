/*
 * nrf24l01.h
 *
 *  Created on: 2021年6月1日
 *      Author: yuejinTan
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "main.h"

void nrfInitRecieve();
int nrfGetData(uint8_t *buffer);
int nrfGetSigOK();

#endif /* INC_NRF24L01_H_ */
