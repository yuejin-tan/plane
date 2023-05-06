/*
 * ctrl.c
 *
 *  Created on: 2021年5月31日
 *      Author: yuejinTan
 */

#include "ctrl.h"
// control state, closing to CTRL_MAX infers better transmition
const int CTRL_MAX=1000;
int ctrlSta = 0;

int16_t pwm_left = 1023;
int16_t pwm_right = 1023;
int16_t servo_left = 4000;
int16_t servo_right = 4000;

uint32_t NRF_buffer[3];

uint8_t ctrl_cmd;

void ctrl_init()
{
    // LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    // LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    // LL_TIM_EnableAllOutputs(TIM1);
    // LL_TIM_EnableCounter(TIM1);

    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM4);
}

void ctrl_refresh()
{
    LL_TIM_OC_SetCompareCH1(TIM1, pwm_left);
    LL_TIM_OC_SetCompareCH2(TIM1, pwm_right);

    LL_TIM_OC_SetCompareCH3(TIM4, servo_left);
    LL_TIM_OC_SetCompareCH4(TIM4, servo_right);
}

void ctrl_decode()
{
    static int lastBagNo = 0;
    uint8_t *pTempPacketNo = (void *)&NRF_buffer[0];
    //int filter, faster
    if (*pTempPacketNo == (lastBagNo + 1) % 256)
    {
        lastBagNo = *pTempPacketNo;
        ctrlSta = (CTRL_MAX * 1 + ctrlSta * 63) >> 6;
    }
    else
    {
        lastBagNo = *pTempPacketNo;
        ctrlSta = (ctrlSta * 63) >> 6;
    }

    ctrl_cmd = NRF_buffer[0] >> 8;

    pwm_left = ((uint16_t *)&NRF_buffer)[1] / 4;
    pwm_right = ((uint16_t *)&NRF_buffer)[2] / 4;
    servo_left = ((uint16_t *)&NRF_buffer)[3] * 4;
    servo_right = ((uint16_t *)&NRF_buffer)[4] * 4;

    return;
}
