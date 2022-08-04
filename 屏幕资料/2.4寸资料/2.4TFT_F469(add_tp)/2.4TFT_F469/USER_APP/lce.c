#include "lcd.h"
#include "stdlib.h"
#include "pic.h"

	   
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 



#define SCLK        7	//PB13--->>TFT --SCL/SCK
#define MOSI        6	//PB15 MOSI--->>TFT --SDA/DIN
#define CS   8       //片选引脚
#define RST  4      //复位引脚

#define LCD1_RST PBout(RST)
#define SPI_MOSI  PCout(MOSI)
#define SPI_SCLK  PCout(SCLK)
#define LCD1_CS  PAout(CS)
//液晶控制口置1操作语句宏定义

#define	SPI_MOSI_SET  	SPI_MOSI=1 //LCD_CTRL->BSRR=SPI_MOSI    
#define	SPI_SCLK_SET  	SPI_SCLK=1 //LCD_CTRL->BSRR=SPI_SCLK  


#define	LCD1_CS_CLR  LCD1_CS=0 //GPIO_TYPE->BSRRH=1<<LCD_CS     //片选端口  	PB11
#define	LCD1_CS_SET  LCD1_CS=1 //GPIO_TYPE->BSRRL=1<<LCD_CS    //片选端口  	PB11
//液晶控制口置0操作语句宏定义

#define	SPI_MOSI_CLR  	SPI_MOSI=0 //LCD_CTRL->BRR=SPI_MOSI    
#define	SPI_SCLK_CLR  	SPI_SCLK=0 //LCD_CTRL->BRR=SPI_SCLK 

#define	LCD1_RST_SET	LCD1_RST=1 //GPIO_TYPE->BSRRL=1<<LCD_RST    //复位			PB12
#define	LCD1_RST_CLR	LCD1_RST=0 //GPIO_TYPE->BSRRH=1<<LCD_RST    //复位			  PB12

void  SPIv_WriteData(u8 Data)
{
	unsigned char i=0;
	for(i=8;i>0;i--)
	{
	  if(Data&0x80)	
	  SPI_MOSI_SET; //输出数据
      else SPI_MOSI_CLR;
	   
      SPI_SCLK_CLR;       
      SPI_SCLK_SET;
      Data<<=1; 
	}
}
void LCD_WR_REG1(u8 data)
{ 
   LCD1_CS_CLR;     
//	 LCD_RS_CLR;
	SPI_MOSI_CLR;
	SPI_SCLK_CLR;       
	SPI_SCLK_SET;
	
   SPIv_WriteData(data);
   LCD1_CS_SET;	
}
/*****************************************************************************
 * @name       :void LCD_WR_DATA11(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA1(u8 data)
{
   LCD1_CS_CLR;
//	 LCD_RS_SET;
		SPI_MOSI_SET;
	SPI_SCLK_CLR;       
	SPI_SCLK_SET;
	
   SPIv_WriteData(data);
   LCD1_CS_SET;
}
void LCD_RESET1(void)
{
	LCD1_RST_CLR;
	HAL_Delay(100);	
	LCD1_RST_SET;
	HAL_Delay(50);
}

void LCD_WR_DATA16(u16 data)
{
   LCD1_CS_CLR;

		SPI_MOSI_SET;
	SPI_SCLK_CLR;       
	SPI_SCLK_SET;
	
	 SPIv_WriteData(data>>8);
   SPIv_WriteData(data);
   LCD1_CS_SET;
}


/*****************************************************************************
 * @name       :void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG1(LCD_Reg);  
	LCD_WR_DATA1(LCD_RegValue);	    		 
}	   



/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG1(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void LCD_ReadRAM_Prepare(void)
 * @date       :2018-11-13 
 * @function   :Read GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG1(lcddev.rramcmd);
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//设置光标位置 
//	Lcd_WriteData_16Bit(POINT_COLOR); 
}



/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(u16 Color)
{
  unsigned int i;//,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	for(i=0;i<lcddev.height*lcddev.width;i++)
	{
 //   for(m=0;m<lcddev.width;m++)
  //  {	
			LCD_WR_DATA1(Color>>8);
	  	LCD_WR_DATA1(Color);
	//	}
	}
} 

void LCD_Color_16Fill(u16 sx,u16 sy,u16 ex,u16 ey, u16 *color)
{
	u16 i,j;			
	u16 width=ex-sx+1; 		//得到填充的宽度
	u16 height=ey-sy+1;		//高度
	LCD_SetWindows(sx,sy,ex,ey);//设置显示窗口
	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		{
					LCD_WR_DATA16(*color);
          color ++;
		}

	}
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);//恢复窗口设置为全屏
	
}

void LCD_Picture(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *color)
{  
	uint16_t i,j;
	uint32_t k=0;
	u16 w=width-x+1; 		//得到填充的宽度
	u16 h=height-y+1;		//高度
  LCD_SetWindows(x,y,width,height);
 	for(i=0;i<h;i++)
	{
		for(j=0;j<w;j++)
		{
			LCD_WR_DATA1(color[k*2]);
			LCD_WR_DATA1(color[k*2+1]);
			k++;
		}
	}           
  LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);//恢复窗口设置为全屏      
}

void LCD_Picture_u16(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint16_t *color)
{  
	uint16_t i,j;
	uint32_t k=0;
  LCD_SetWindows(x,y,width,height);
 	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA1(color[k]>>8);
			LCD_WR_DATA1(color[k]);
			k++;
		}
	}           
        
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD1_RST_CLR;
	HAL_Delay(100);	
	LCD1_RST_SET;
	HAL_Delay(50);
}

/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  


	LCD_RESET1(); //LCD 复位	
	HAL_Delay(100);
	//*************ILI9481 Start Initial Sequence **********//		
  LCD_WR_REG1(0xF0);
	LCD_WR_DATA1(0xC3);
	LCD_WR_REG1(0xF0);
	LCD_WR_DATA1(0x96);
	LCD_WR_REG1(0x36);
	LCD_WR_DATA1(0x68);	
	LCD_WR_REG1(0x3A);
	LCD_WR_DATA1(0x05);	
	LCD_WR_REG1(0xB0);
	LCD_WR_DATA1(0x80);	
	LCD_WR_REG1(0xB6);
	LCD_WR_DATA1(0x00);
	LCD_WR_DATA1(0x02);	
	LCD_WR_REG1(0xB5);
	LCD_WR_DATA1(0x02);
	LCD_WR_DATA1(0x03);
	LCD_WR_DATA1(0x00);
	LCD_WR_DATA1(0x04);
	LCD_WR_REG1(0xB1);
	LCD_WR_DATA1(0x80);	
	LCD_WR_DATA1(0x10);	
	LCD_WR_REG1(0xB4);
	LCD_WR_DATA1(0x00);
	LCD_WR_REG1(0xB7);
	LCD_WR_DATA1(0xC6);
	LCD_WR_REG1(0xC5);
	LCD_WR_DATA1(0x24);
	LCD_WR_REG1(0xE4);
	LCD_WR_DATA1(0x31);
	LCD_WR_REG1(0xE8);
	LCD_WR_DATA1(0x40);
	LCD_WR_DATA1(0x8A);
	LCD_WR_DATA1(0x00);
	LCD_WR_DATA1(0x00);
	LCD_WR_DATA1(0x29);
	LCD_WR_DATA1(0x19);
	LCD_WR_DATA1(0xA5);
	LCD_WR_DATA1(0x33);
	LCD_WR_REG1(0xC2);
	LCD_WR_REG1(0xA7);
	
	LCD_WR_REG1(0xE0);
	LCD_WR_DATA1(0xF0);
	LCD_WR_DATA1(0x09);
	LCD_WR_DATA1(0x13);
	LCD_WR_DATA1(0x12);
	LCD_WR_DATA1(0x12);
	LCD_WR_DATA1(0x2B);
	LCD_WR_DATA1(0x3C);
	LCD_WR_DATA1(0x44);
	LCD_WR_DATA1(0x4B);
	LCD_WR_DATA1(0x1B);
	LCD_WR_DATA1(0x18);
	LCD_WR_DATA1(0x17);
	LCD_WR_DATA1(0x1D);
	LCD_WR_DATA1(0x21);

	LCD_WR_REG1(0XE1);
	LCD_WR_DATA1(0xF0);
	LCD_WR_DATA1(0x09);
	LCD_WR_DATA1(0x13);
	LCD_WR_DATA1(0x0C);
	LCD_WR_DATA1(0x0D);
	LCD_WR_DATA1(0x27);
	LCD_WR_DATA1(0x3B);
	LCD_WR_DATA1(0x44);
	LCD_WR_DATA1(0x4D);
	LCD_WR_DATA1(0x0B);
	LCD_WR_DATA1(0x17);
	LCD_WR_DATA1(0x17);
	LCD_WR_DATA1(0x1D);
	LCD_WR_DATA1(0x21);

  LCD_WR_REG1(0X36);
	LCD_WR_DATA1(0xEC);
	LCD_WR_REG1(0xF0);
	LCD_WR_DATA1(0xC3);
	LCD_WR_REG1(0xF0);
	LCD_WR_DATA1(0x69);
	LCD_WR_REG1(0X13);
	LCD_WR_REG1(0X11);
	LCD_WR_REG1(0X29);
	
  LCD_direction(USE_HORIZONTAL);//设置LCD显示方向
	LCD_LED=1;//点亮背光	 
	LCD_Clear(RED);//清全屏白色
	
//	LCD_Clear(BLUE);//清全屏白色
//	
//	LCD_Clear(YELLOW);//清全屏白色
//	
//	LCD_Clear(GREEN);//清全屏白色
//	
////LCD_Picture(0,0,320,480,gImage_520);
////	LCD_Picture(0,0,320,480,gImage);
	
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG1(lcddev.setxcmd);	
	LCD_WR_DATA1(xStar>>8);
	LCD_WR_DATA1(0x00FF&xStar);		
	LCD_WR_DATA1(xEnd>>8);
	LCD_WR_DATA1(0x00FF&xEnd);

	LCD_WR_REG1(lcddev.setycmd);	
	LCD_WR_DATA1(yStar>>8);
	LCD_WR_DATA1(0x00FF&yStar);		
	LCD_WR_DATA1(yEnd>>8);
	LCD_WR_DATA1(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//开始写入GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(u8 direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
			lcddev.rramcmd=0x2E;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3)|(1<<6));
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<5));
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<4));
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<6)|(1<<5)|(1<<4));
		break;	
		default:break;
	}		
}	 


