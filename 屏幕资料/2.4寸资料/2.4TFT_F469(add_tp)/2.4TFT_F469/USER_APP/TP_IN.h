#ifndef __TP_IN_H__
#define __TP_IN_H__
#include "main.h"
#include "sys.h"
//I2C��д����	
//#define FT_CMD_WR 	  0X54    	//д����
//#define FT_CMD_RD 		0X55		//����

#define FT_CMD_WR 	  0X70    	//д����
#define FT_CMD_RD 		0X71		//����

//IO�ڶ���
#define FT6336_SDA_H() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET)
#define FT6336_SDA_L() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET)

#define FT6336_SCL_H() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET)
#define FT6336_SCL_L() 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET)

#define FT6336_RST_H()	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET)
#define FT6336_RST_L()	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET)

#define SDA_IN()   {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	 //����ģʽ����������ģʽ
#define SDA_OUT()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;}	 //ͨ���������������ٶ�50MHZ

#define Is_SDA_IN  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) //

#define TP_PRES_DOWN 0x80  //����������	
#define TP_COORD_UD  0x40  //����������±��

//������������ݽṹ�嶨��
typedef struct			
{
	u8 TouchSta;	//���������b7:����1/�ɿ�0; b6:0û�а�������/1�а�������;bit5:������bit4-bit0�����㰴����Ч��־����ЧΪ1���ֱ��Ӧ������5-1��
	u16 x[5];		//֧��5�㴥������Ҫʹ��5������洢����������
	u16 y[5];
	
}TouchPointRefTypeDef;
extern TouchPointRefTypeDef TPR_Structure;

#define I2C_ADDR_FT6336 0x38

//FT6236 ���ּĴ������� 
#define FT_DEVIDE_MODE 			0x00   		//FT6236ģʽ���ƼĴ���
#define FT_REG_NUM_FINGER       0x02		//����״̬�Ĵ���

#define FT_TP1_REG 				0X03	  	//��һ�����������ݵ�ַ
#define FT_TP2_REG 				0X09		//�ڶ������������ݵ�ַ
#define FT_TP3_REG 				0X0F		//���������������ݵ�ַ
#define FT_TP4_REG 				0X15		//���ĸ����������ݵ�ַ
#define FT_TP5_REG 				0X1B		//��������������ݵ�ַ  
 

#define	FT_ID_G_LIB_VERSION		0xA1		//�汾		
#define FT_ID_G_MODE 			0xA4   		//FT6236�ж�ģʽ���ƼĴ���
#define FT_ID_G_THGROUP			0x80   		//������Чֵ���üĴ���
#define FT_ID_G_PERIODACTIVE	0x88   		//����״̬�������üĴ���  
#define Chip_Vendor_ID          0xA3        //оƬID(0x36)
#define ID_G_FT6236ID			0xA8		//0x11


#define FT6336_ADDR_DEVICE_MODE 	0x00
#define FT6336_ADDR_TD_STATUS 		0x02
#define FT6336_ADDR_TOUCH1_EVENT 	0x03
#define FT6336_ADDR_TOUCH1_ID 		0x05
#define FT6336_ADDR_TOUCH1_X 		0x03
#define FT6336_ADDR_TOUCH1_Y 		0x05

#define FT6336_ADDR_TOUCH2_EVENT 	0x09
#define FT6336_ADDR_TOUCH2_ID 		0x0B
#define FT6336_ADDR_TOUCH2_X 		0x09
#define FT6336_ADDR_TOUCH2_Y 		0x0B

#define FT6336_ADDR_FIRMARE_ID 		0xA6


void I2C_Start(void);
void I2C_End(void);
void i2c_write_byte(unsigned char temp);
unsigned char i2c_read_byte(void);
unsigned char i2c_read_keep_byte(void);


u8 FT6336_WR_Reg(u8 reg,u8 *buf,u8 len);
void FT6336_RD_Reg(u8 reg,u8 *buf,u8 len);
void FT6336_Init(void);

unsigned char i2c_read_addr_byte(unsigned char device_addr,unsigned char read_addr);
void i2c_read_addr_str(unsigned char device_addr,unsigned char read_addr,unsigned char read_amount,unsigned char *read_buf);
void i2c_write_addr_byte(unsigned char device_addr,unsigned char write_addr,unsigned char write_dat);
unsigned int i2c_read_addr_int(unsigned char device_addr,unsigned char read_addr);
void i2c_write_addr_str(unsigned char device_addr,unsigned char write_addr,unsigned char write_amount,unsigned char *write_buf);

#endif