#ifndef __LCD_H__
#define __LCD_H__
#include "main.h"

/* use GPIO define */
#define LCD_XRES_SET()			HAL_GPIO_WritePin(GPIOC, RESX_Pin, GPIO_PIN_SET)
#define LCD_XRES_RESET()		HAL_GPIO_WritePin(GPIOC, RESX_Pin, GPIO_PIN_RESET)

#define LCD_DCX_SET()			HAL_GPIO_WritePin(GPIOC, DXC_Pin, GPIO_PIN_SET)
#define LCD_DCX_RESET()			HAL_GPIO_WritePin(GPIOC, DXC_Pin, GPIO_PIN_RESET)

#define SPI_CS_SET()			HAL_GPIO_WritePin(GPIOC, CSX_Pin, GPIO_PIN_SET)
#define SPI_CS_RESET()		HAL_GPIO_WritePin(GPIOC, CSX_Pin, GPIO_PIN_RESET)

	 

//»­±ÊÑÕÉ«
#define WHITE         	 0xFF
#define BLACK         	 0x00	
#define GRAY         	 		0x07F0
#define RED           	 0xE0
#define GREEN         		0x1C
#define BLUE         	 	0x03  
#define BRED            0XE3
#define GRED 			 			0XFC
#define GBLUE			 			0X1F

void LCD_Color_Fill_u16(uint16_t sx,uint16_t sy,uint16_t fx,uint16_t fy,uint16_t *color);
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t fx,uint16_t fy,const uint8_t *color);
void lcd_show_pic(uint16_t sx, uint16_t sy, uint16_t xpix, uint16_t ypix,int8_t *color);
void set_window_xy(uint16_t xstart, uint16_t ystart, uint16_t xend, uint16_t yend);
void LCD_pic_dma(uint16_t sx,uint16_t sy,uint16_t fx,uint16_t fy, uint8_t *color);
void LcdTask();
#endif



