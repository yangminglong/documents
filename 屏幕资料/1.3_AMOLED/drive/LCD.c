#include "LCD.h"
#include "spi.h"
#include "PIC.h"
//#include "cmsis_os.h"

//write cmd
void lcd_write_cmd(uint8_t cmd)
{
    LCD_DCX_RESET();
    HAL_SPI_Transmit(&hspi6, &cmd, 1, 1000);
    LCD_DCX_SET();
}

//write data
void lcd_write_data(uint8_t data)
{
    HAL_SPI_Transmit(&hspi6, &data, 1, 1000);
}

//write RGB565 color
void lcd_write_color(uint16_t color)
{
    unsigned char DataL;
    unsigned char DataH;
    DataL = color;
    DataH = color >> 8;
    HAL_SPI_Transmit(&hspi6, &DataH, 1, 1000);
    HAL_SPI_Transmit(&hspi6, &DataL, 1, 1000);
}


//the cmd for a data
void lcd_write_cmd_1data(uint8_t cmd, uint8_t data)
{
    LCD_DCX_RESET();
    HAL_SPI_Transmit(&hspi6, &cmd, 1, 1000);
    LCD_DCX_SET();
    HAL_SPI_Transmit(&hspi6, &data, 1, 1000);
}

//the cmd write for more data
void lcd_write_cmd_data(uint8_t cmd, uint8_t *data, uint8_t len)
{
    uint8_t cnt;
    LCD_DCX_RESET();
    HAL_SPI_Transmit(&hspi6, &cmd, 1, 1000);
    LCD_DCX_SET();
    for(cnt = 0; cnt < len; cnt++)
        HAL_SPI_Transmit(&hspi6, &data[cnt], 1, 1000);
}

//sleep
void lcd_sleep_in(void)
{
    lcd_write_cmd(0x10);
}
void lcd_sleep_out(void)
{
    lcd_write_cmd(0x11);
}
//display
void lcd_display_off(void)
{
    lcd_write_cmd(0x28);
}
void lcd_display_on(void)
{
    lcd_write_cmd(0x29);
}
//reset
void lcd_reset(void)
{
    LCD_XRES_RESET();
    HAL_Delay(1);
    LCD_XRES_SET();
    HAL_Delay(10);
}
uint8_t id[3];
void lcd_Init(void)
{
    uint8_t i;
    uint8_t add[4];
    uint8_t daa[10];

    lcd_reset();
    //	  lcd_read_cmd_data(0x04, id, 3);
    add[0] = 0;
    add[1] = 0;
    add[2] = 0x01;
    add[3] = 0xc5;

    lcd_write_cmd_1data(0xfe, 0x01); // page 0
    lcd_write_cmd_1data(0x6c, 0x0a); // mipi turn off
    lcd_write_cmd_1data(0x04, 0xa0);// 开启 spi 写 ram，此04必需写0xA0才能打开SPIR
    lcd_write_cmd_1data(0xfe, 0x05);//Page4
    lcd_write_cmd_1data(0x05, 0x00);
    lcd_write_cmd_1data(0xfe, 0x00);//User Command
    lcd_write_cmd_1data(0x35, 0x00);
    lcd_write_cmd_1data(0x36, 0x00);//RM6716X_MADCTL_MY
    lcd_write_cmd_1data(0x51, 0xFF);//RM6716X_CMD_SET_BRIGHTNESS
    lcd_write_cmd_1data(0x53, 0x20);//close dimming
    lcd_write_cmd_1data(0xc4, 0x80);
    lcd_write_cmd_1data(0x3a, 0x75);//RM6716X_RGB_FORMAT_565

    lcd_write_cmd_data(0x2A, add, 4);//No offsets set window

    lcd_write_cmd_data(0x2B, add, 4);//No offsets set window

    lcd_write_cmd(0x11);            // sleep out
    HAL_Delay(20);
    lcd_write_cmd(0x29);//display on
    HAL_Delay(20);

    lcd_write_cmd(0x2C);//write Gram
    HAL_Delay(20);
}

//set display zone
//#define START_H_ADD 	0
//#define START_V_ADD 	0
//#define MAX_COLUMN 		454
//#define MAX_ROW 			454
int16_t START_H_ADD =	10;                                                                                                                                                                                                                                                                                                                                                         ;
int16_t START_V_ADD =	-20;
int16_t START_X_ADD =	10;
int16_t MAX_COLUMN =	-20;
int16_t MAX_ROW 	=		454;
uint16_t s_x, s_y, f_x, f_y;
void set_window_xy(uint16_t xstart, uint16_t ystart, uint16_t xend, uint16_t yend)
{
    uint8_t add[4];

    xstart += START_H_ADD;
    ystart += START_V_ADD;
    xend += START_X_ADD;
    yend += MAX_COLUMN;

    add[0] = (uint8_t)(xstart >> 8);
    add[1] = (uint8_t)xstart;
    add[2] = (uint8_t)(xend >> 8);
    add[3] = (uint8_t)xend;
    lcd_write_cmd_data(0x2A, add, 4);

    add[0] = (uint8_t)(ystart >> 8);
    add[1] = (uint8_t)ystart;
    add[2] = (uint8_t)(yend >> 8);
    add[3] = (uint8_t)yend;
    lcd_write_cmd_data(0x2B, add, 4);

    lcd_write_cmd(0x2C);
}
//clear full display zone
//color:rgb332
void LCD_Clear(uint16_t color)
{
    uint16_t i, j;
    set_window_xy(0, 0, MAX_COLUMN, MAX_ROW);
    for(i = 0; i < MAX_COLUMN; i++)
    {
        for(j = 0; j < MAX_ROW; j++)
        {
            lcd_write_data((uint8_t)(color >> 8));
            lcd_write_data((uint8_t)color);
        }
    }
}

//write a zone at the specified location use color
//start(sx,sy):sx,sy
//zone(fx,fy):width,lenth

void LCD_Color_Fill(uint16_t sx, uint16_t sy, uint16_t fx, uint16_t fy, const uint8_t *color)
{
    uint16_t i, j;

    set_window_xy(sx, sy, fx + sx, fy + sy);

    for(i = 0; i < fx; i++)
    {
        for(j = 0; j < fy; j++)
        {
            HAL_SPI_Transmit(&hspi6, (uint8_t *)color, 1, 1000);
            HAL_SPI_Transmit(&hspi6, (uint8_t *)color + 1, 1, 1000);
            color += 2;
        }
    }

    lcd_write_cmd(0x2C);
}

void LCD_Color_Fill_u16(uint16_t sx, uint16_t sy, uint16_t fx, uint16_t fy, uint16_t *color)
{
    uint16_t i, j;
    set_window_xy(sx, sy, fx + sx, fy + sy);
    for(i = 0; i < fx; i++)
    {
        for(j = 0; j < fy; j++)
        {
            lcd_write_color(*color);
            color += 1;
        }
    }
}

//write a zone at the specified location use color
//start(sx,sy):sx,sy
//zone(fx,fy):width,lenth
void LCD_Color_16Fill(uint16_t sx, uint16_t sy, uint16_t fx, uint16_t fy, const uint16_t color)
{
    uint16_t i, j;
    unsigned char DataL;
    unsigned char DataH;
    DataL = color;
    DataH = color >> 8;
    set_window_xy(sx, sy, fx + sx, fy + sy);
    for(i = 0; i < fx; i++)
    {
        for(j = 0; j < fy; j++)
        {
            HAL_SPI_Transmit(&hspi6, &DataH, 1, 1000);
            HAL_SPI_Transmit(&hspi6, &DataL, 1, 1000);
        }
    }
    //
}

//显存定义
//显存总大小 240*240*(16bit) = 240*240*2 个字节
#define LCD_TOTAL_BUF_SIZE	(454*454*2)
//因为直接定义显存太大了，所以定义其 1/100 轮流刷新
#define LCD_Buf_Size 25764*2
uint8_t lcd_buf[LCD_Buf_Size];
void LCD_Color_16Fill_dma2(uint16_t sx, uint16_t sy, uint16_t fx, uint16_t fy, const uint16_t color)
{

    uint32_t j, i;
    uint8_t data[2] = {0};  //color是16bit的，每个像素点需要两个字节的显存

    /* 将16bit的color值分开为两个单独的字节 */
    data[0] = color >> 8;
    data[1] = color;
    /* 显存的值需要逐字节写入显存 */
    for(j = 0; j < LCD_Buf_Size / 2; j++)
    {
        lcd_buf[j * 2] =  data[0];
        lcd_buf[j * 2 + 1] =  data[1];
    }
    /* 显存更新到 Flash */
    set_window_xy(0, 0, 454, 454);
    /* 循环将显存缓冲区的数据循环写入到LCD */

    for(i = 0; i < 8; i++)
    {
        HAL_SPI_Transmit_DMA(&hspi6, lcd_buf, 51529);
        HAL_Delay(10);//必须加延迟
    }

}

void LCD_pic_dma(uint16_t sx, uint16_t sy, uint16_t fx, uint16_t fy, uint8_t *color)
{
    uint16_t i, j, x;
    int16_t w = (fx - sx + 1);
    int16_t h = (fy - sy + 1);
    uint32_t size = w * h * 2;
    set_window_xy(sx, sy, fx, fy);
    HAL_SPI_Transmit_DMA(&hspi6, color, size);
    //	  if(size>40000)
    //		{
    //			HAL_Delay(5);
}


#define xcolor 0xFFFF
#define HEI 0x0000
#define LAN 0x867f
#define LU 0x9772
#define HUANG 0xffe0
#define JING 0xfea0
void LcdTask()
{
    uint8_t rgb[1] = {0xfd};
    lcd_Init();

    //	LCD_Color_Fill(0, 0, 200, 100, gImage_2);
    //	LCD_Color_Fill(100, 100, 240, 240, gImage_1);

    LCD_Color_16Fill(0, 0, 400, 400, HUANG);
    LCD_Color_16Fill(0, 0, 450, 450, LAN);
    LCD_Color_16Fill(0, 0, 450, 450, HUANG);

    //    LCD_pic_dma(0, 0, 200, 100, gImage_2);


}

