/*
 * nrf24l01.c
 *
 *  Created on: 2021年6月1日
 *      Author: yuejinTan
 */

#include "nrf24l01.h"

const uint8_t NRF24_R_REG = 0b00000000;              //读配置寄存器,低5位为寄存器地址
const uint8_t NRF24_W_REG = 0b00100000;              //写配置寄存器,低5位为寄存器地址
const uint8_t NRF24_R_RX_PAYLOAD = 0b01100001;       //读RX有效数据,1~32字节
const uint8_t NRF24_W_TX_PAYLOAD = 0b10100000;       //写TX有效数据,1~32字节
const uint8_t NRF24_FLUSH_TX = 0b11100001;           //清除TX FIFO寄存器.发射模式下用
const uint8_t NRF24_FLUSH_RX = 0b11100010;           //清除RX FIFO寄存器.接收模式下用.如果需要回ACK，则不能在回ACK操作完成前进行清空IFO，否则视为通信失败。
const uint8_t NRF24_REUSE_TX_PL = 0b11100011;        //重新使用上一包数据,CE为高,数据包被不断发送.清空TX FIFO或对FIFO写入新的数据后不能使用该命令。
const uint8_t NRF24_R_RX_PL_WID = 0b01100000;        //读取收到的数据字节数。
const uint8_t NRF24_W_ACK_PAYLOAD = 0b10101000;      //适用于接收方，通过PIPExxx将数据通过ACK的形式发出去，最多允许三帧数据存于FIFO中。
const uint8_t NRF24_W_TX_PAYLOAD_NOACK = 0b10110000; //适用于发射模式，使用这个命令同时需要将AUTOACK位置1。
const uint8_t NRF24_NOP = 0b11111111;                //空操作,可以用来读状态寄存器

const uint8_t NRF24_REG_CONFIG = 0x00;     //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
const uint8_t NRF24_REG_EN_AA = 0x01;      //使能自动应答功能  bit0~5,对应通道0~5
const uint8_t NRF24_REG_EN_RXADDR = 0x02;  //接收地址允许,bit0~5,对应通道0~5
const uint8_t NRF24_REG_SETUP_AW = 0x03;   //设置地址宽度(所有数据通道):bit1:0:01,3字节;10,4字节;11,5字节;
const uint8_t NRF24_REG_SETUP_RETR = 0x04; //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
const uint8_t NRF24_REG_RF_CH = 0x05;      //RF通道,bit6:0,工作通道频率;
const uint8_t NRF24_REG_RF_SETUP = 0x06;   //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
const uint8_t NRF24_REG_STATUS = 0x07;     //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发;bit5:数据发送完成中断;bit6:接收数据中断;

const uint8_t NRF24_REG_OBSERVE_TX = 0x08;  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
const uint8_t NRF24_REG_RSSI = 0x09;        //载波检测寄存器,bit0,载波检测;
const uint8_t NRF24_REG_RX_ADDR_P0 = 0x0A;  //数据通道0接收地址,最大长度5个字节,低字节在前
const uint8_t NRF24_REG_RX_ADDR_P1 = 0x0B;  //数据通道1接收地址,最大长度5个字节,低字节在前
const uint8_t NRF24_REG_RX_ADDR_P2 = 0x0C;  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
const uint8_t NRF24_REG_RX_ADDR_P3 = 0x0D;  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
const uint8_t NRF24_REG_RX_ADDR_P4 = 0x0E;  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
const uint8_t NRF24_REG_RX_ADDR_P5 = 0x0F;  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
const uint8_t NRF24_REG_TX_ADDR = 0x10;     //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
const uint8_t NRF24_REG_RX_PW_P0 = 0x11;    //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
const uint8_t NRF24_REG_RX_PW_P1 = 0x12;    //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
const uint8_t NRF24_REG_RX_PW_P2 = 0x13;    //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
const uint8_t NRF24_REG_RX_PW_P3 = 0x14;    //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
const uint8_t NRF24_REG_RX_PW_P4 = 0x15;    //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
const uint8_t NRF24_REG_RX_PW_P5 = 0x16;    //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
const uint8_t NRF24_REG_FIFO_STATUS = 0x17; //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留;bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
const uint8_t NRF24_REG_DYNPD = 0x1c;       //使能动态负载长度,bit0~5,对应通道0~5
const uint8_t NRF24_REG_FEATURE = 0x1d;     //特征寄存器;bit0:使能命令W_TX_PAYLOAD_NOACK;bit1:使能ACK负载(带负载数据的ACK包);bit2:使能动态负载长度

volatile uint8_t nrf24Sta = 0;

void nrfDelayUs()
{
    LL_TIM_EnableCounter(TIM3);
    while (LL_TIM_IsEnabledCounter(TIM3))
    {
        // wait
    }
}

uint8_t nrfRegAccess(uint8_t REG, uint8_t data)
{
    uint8_t ret;
    nrfDelayUs();
    LL_GPIO_ResetOutputPin(CS_24L_GPIO_Port, CS_24L_Pin);
    nrfDelayUs();
    LL_SPI_TransmitData8(SPI1, REG);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
    {
        // wait
    }
    nrf24Sta = LL_SPI_ReceiveData8(SPI1);

    LL_SPI_TransmitData8(SPI1, data);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
    {
        // wait
    }
    ret = LL_SPI_ReceiveData8(SPI1);
    nrfDelayUs();
    LL_GPIO_SetOutputPin(CS_24L_GPIO_Port, CS_24L_Pin);
    return ret;
}

void nrfInitRecieve()
{
    LL_SPI_Enable(SPI1);
    LL_mDelay(10);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_CONFIG, 0b1011);
    LL_mDelay(10);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_EN_AA, 0b0);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_EN_RXADDR, 0b11);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_SETUP_AW, 0b01);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_SETUP_RETR, 0b0);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_RF_CH, 0b10);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_RF_SETUP, 0b100111);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_RX_PW_P0, 10);
    nrfRegAccess(NRF24_W_REG | NRF24_REG_RX_PW_P1, 10);

    LL_GPIO_ResetOutputPin(CS_24L_GPIO_Port, CS_24L_Pin);
    LL_SPI_TransmitData8(SPI1, NRF24_W_REG | NRF24_REG_RX_ADDR_P1);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
    {
        // wait
    }
    nrf24Sta = LL_SPI_ReceiveData8(SPI1);

    LL_SPI_TransmitData8(SPI1, 0xee);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
    {
        // wait
    }
    LL_SPI_ReceiveData8(SPI1);
    LL_SPI_TransmitData8(SPI1, 0xff);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
    {
        // wait
    }
    LL_SPI_ReceiveData8(SPI1);
    LL_SPI_TransmitData8(SPI1, 0xc0);
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
    {
        // wait
    }
    LL_SPI_ReceiveData8(SPI1);
    nrfDelayUs();
    LL_GPIO_SetOutputPin(CS_24L_GPIO_Port, CS_24L_Pin);

    nrfDelayUs();
    LL_GPIO_SetOutputPin(CE_24L_GPIO_Port, CE_24L_Pin);

#if 0
   volatile uint8_t aa1 = nrfRegAccess(NRF24_R_REG | NRF24_REG_CONFIG, NRF24_NOP);
   volatile uint8_t aa2 = nrfRegAccess(NRF24_R_REG | NRF24_REG_EN_AA, NRF24_NOP);
   volatile uint8_t aa3 = nrfRegAccess(NRF24_R_REG | NRF24_REG_EN_RXADDR, NRF24_NOP);
   volatile uint8_t aa4 = nrfRegAccess(NRF24_R_REG | NRF24_REG_SETUP_AW, NRF24_NOP);
   volatile uint8_t aa5 = nrfRegAccess(NRF24_R_REG | NRF24_REG_SETUP_RETR, NRF24_NOP);
   volatile uint8_t aa6 = nrfRegAccess(NRF24_R_REG | NRF24_REG_RF_CH, NRF24_NOP);
   volatile uint8_t aa7 = nrfRegAccess(NRF24_R_REG | NRF24_REG_RF_SETUP, NRF24_NOP);
   volatile uint8_t aa8 = nrfRegAccess(NRF24_R_REG | NRF24_REG_RX_PW_P1, NRF24_NOP);
   volatile uint8_t aa9 = nrfRegAccess(NRF24_R_REG | NRF24_REG_EN_AA, NRF24_NOP);
#endif

    LL_mDelay(10);
}

int nrfGetData(uint8_t *buffer)
{
    nrfRegAccess(NRF24_W_REG | NRF24_REG_STATUS, 0b1110000);
    if ((nrf24Sta & 0b1110) != 0b1110)
    {
        nrfDelayUs();
        LL_GPIO_ResetOutputPin(CS_24L_GPIO_Port, CS_24L_Pin);
        nrfDelayUs();
        LL_SPI_TransmitData8(SPI1, NRF24_R_RX_PAYLOAD);
        while (LL_SPI_IsActiveFlag_BSY(SPI1))
        {
            // wait
        }
        nrf24Sta = LL_SPI_ReceiveData8(SPI1);
        for (int i = 0; i < 10; i++)
        {

            LL_SPI_TransmitData8(SPI1, NRF24_NOP);
            while (LL_SPI_IsActiveFlag_BSY(SPI1))
            {
                // wait
            }
            buffer[i] = LL_SPI_ReceiveData8(SPI1);
        }
        nrfDelayUs();
        LL_GPIO_SetOutputPin(CS_24L_GPIO_Port, CS_24L_Pin);
        return 1;
    }
    return 0;
}

int nrfGetSigOK()
{
    return (nrfRegAccess(NRF24_R_REG | NRF24_REG_RSSI, NRF24_NOP));
}
