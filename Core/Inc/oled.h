#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#define OLED_MODE 0
#define SIZE 8
#define XLevelL 0x00
#define XLevelH 0x10
#define Max_Column 128
#define Max_Row 64
#define Brightness 0xFF
#define X_WIDTH 128
#define Y_WIDTH 64
//-----------------OLED IIC端口定义----------------

#define OLED_CMD 0  //写命令
#define OLED_DATA 1 //写数据

#define IIC_SLAVE_ADDR 0x78 //IIC slave device address

//OLED控制用函数
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(unsigned dat);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t Char_Size);
void OLED_ShowString(uint8_t x, uint8_t y, char *p, uint8_t Char_Size);
void OLED_Set_Pos(unsigned char x, unsigned char y);

#endif
