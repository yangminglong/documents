#include "string.h"
#include "gpio.h"
#include "tp_in.h"

void delay_us(uint32_t us) 
{
    uint32_t i = 0;
    for(i=0; i<us; i++) {
        uint8_t a = 2;
        while(a--);
    }
}
/* 
**��������FT6236_Init
**�����������
**����ֵ����
**���ܣ���ʼ��FT6236����
*/  
u8 id = 0;
u8 panel_id = 0;
u8 dev_ode = 0;
u8 code_ode = 0;
  void FT6336_Init(void)
{
	u8 temp,KK;
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOC_CLK_ENABLE();   //ʹ��GPIOBʱ��
    __HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOBʱ��
    
    //��ʼ������
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FAST;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	

	

	FT6336_RST_L();
	HAL_Delay(50);
	FT6336_RST_H();
	HAL_Delay(10);
	
	FT6336_SDA_H();
	HAL_Delay(10);
	FT6336_SCL_H();
	HAL_Delay(10);
	temp=0x00;
	FT6336_WR_Reg(FT_DEVIDE_MODE,&temp,1);	//������������ģʽ 
	FT6336_RD_Reg(FT_DEVIDE_MODE,&dev_ode,1);	
	
		FT6336_RD_Reg(0xA3,&id,1);	
		FT6336_RD_Reg(0xA8,&panel_id,1);	
		FT6336_RD_Reg(0xAf,&code_ode,1);	
		
    
		
		
 	temp=22;								//������Чֵ��22��ԽСԽ����	
 	FT6336_WR_Reg(FT_ID_G_THGROUP,&temp,1);	//���ô�����Чֵ
	FT6336_RD_Reg(FT_ID_G_THGROUP,&KK,1);
 	temp=14;								//�������ڣ�����С��12�����14
 	FT6336_WR_Reg(FT_ID_G_PERIODACTIVE,&temp,1); 
	FT6336_RD_Reg(FT_ID_G_PERIODACTIVE,&KK,1);
/******************************************************/
}




//-------------------------------�ָ�Ʋ���-----------------------
/****************************************************
* �������� ��
* ��    �� ����Ƭ��������ʼ�ź�
* ��ڲ��� ����
* ���ز��� ����
* ע������ ��
*****************************************************/
void I2C_Start(void)
{
	SDA_OUT();     //sda�����
	FT6336_SDA_H();	  	  
	FT6336_SCL_H();
	delay_us(10);   //8ͨ��ʱ,5us����,16ͨ��ʱ,�����������½�,������Ҫ8us,Ϊ�˱�֤û������10us
 	FT6336_SDA_L();//START:when i2c is high,DATA change form high to low 
	delay_us(5);
	FT6336_SCL_L();//ǯסI2C���ߣ�׼�����ͻ�������� 
	delay_us(10);
}

/****************************************************
* �������� ��
* ��    �� ����Ƭ������ֹͣ�ź�
* ��ڲ��� ����
* ���ز��� ����
* ע������ ��
*****************************************************/
void I2C_End(void)
{
//	SDA_OUT();     		//sda�����
//	FT6336_SDA_L();		
//	delay_us(5);	
//	FT6336_SCL_H();		//SCL��С�ߵ�ƽ����:0.6us		
////	delay_us(4);		//ֹͣ�źŵ���С����ʱ��:0.6us
////	FT6336_SDA_L();	
//	delay_us(10);
//	FT6336_SDA_H();		//SCL�ߵ�ƽ�ڼ䣬SDA��һ�������ر�ʾֹͣ�ź�
//	delay_us(5);		
	
	SDA_OUT();     		//sda�����
	FT6336_SCL_L();		//SCL��С�ߵ�ƽ����:0.6us		
	FT6336_SDA_L();		
	delay_us(5);	
	FT6336_SCL_H();		//SCL��С�ߵ�ƽ����:0.6us		
	delay_us(4);		//ֹͣ�źŵ���С����ʱ��:0.6us
	FT6336_SDA_H();		//SCL�ߵ�ƽ�ڼ䣬SDA��һ�������ر�ʾֹͣ�ź�
	delay_us(5);	
}


/****************************************************
* �������� ��
* ��    �� ����Ƭ������Ӧ���ź�
* ��ڲ��� ����
* ���ز��� ����
* ע������ ����Ƭ����1B���ݺ���һ��Ӧ���ź�
*****************************************************/
void FT6336_McuACK(void)							
{
	FT6336_SCL_L();
	SDA_OUT();     		//sda�����	
	FT6336_SDA_L();
	delay_us(5);																	
	FT6336_SCL_H();		//SCL��С�ߵ�ƽ����:0.6us
	delay_us(10);
	FT6336_SCL_L();	//SCL��С�͵�ƽ����:1.2us
	delay_us(10);
	
	FT6336_SDA_H();
}


/****************************************************
* �������� ��
* ��    �� ����Ƭ�����ͷ�Ӧ���ź�
* ��ڲ��� ����
* ���ز��� ����
* ע������ ����Ƭ��������ֹͣǰ����һ����Ӧ���ź�
*****************************************************/
void FT6336_McuNACK(void)
{
	FT6336_SCL_L();
	SDA_OUT();     				//sda�����	
	FT6336_SDA_H();
	delay_us(5);																	
	FT6336_SCL_H();				//SCL��С�ߵ�ƽ����:0.6us
	delay_us(10);
	FT6336_SCL_L();			//SCL��С�͵�ƽ����:1.2us
	delay_us(10);
}

/****************************************************
* �������� ��
* ��    �� ����Ƭ�����FT6236������Ӧ���ź�
* ��ڲ��� ����
* ���ز��� ��1������Ӧ��ʧ��
			 0������Ӧ��ɹ�
* ע������ ����Ƭ��д1����ַ/���ݺ���
			 ȫ�ֱ���RevAckF:�յ�FT6236Ӧ���źŵı�־λ,Ϊ0��ʾ�յ�
*****************************************************/
u8 FT6336_CheckAck(void)							
{
	u8 ucErrTime=0;
	u8 redata;
	
	FT6336_SDA_H();
//	delay_us(5);  
	SDA_IN();  //SDA����Ϊ����
	FT6336_SCL_H();			//ʹSDA��������Ч;SCL��С�ߵ�ƽ����:0.6us
	delay_us(10);
	while(Is_SDA_IN != 0)
	{	
		ucErrTime++;
		if(ucErrTime>250)		//��Ӧ��
		{
			I2C_End();	
			return 1;
		}
	}
	FT6336_SCL_L();
	delay_us(10);
//	SDA_IN();
//	FT6336_SDA_H();
	return 0;
	
}

/****************************************************
* �������� ��
* ��    �� ����Ƭ����IIC���߷���1B�ĵ�ַ/����
* ��ڲ��� �������͵�1B��ַ/����
* ���ز��� ����
* ע������ ������һ�����������ݷ��͹���;������˳���ǴӸߵ���
*****************************************************/
void FT6336_WrOneByte(u8 dat)						
{
	u8 i;						
	SDA_OUT();     				//sda�����	
	FT6336_SCL_L();				//����ʱ�ӿ�ʼ���ݴ���
	for(i = 0; i < 8; i++)		//8λ1B��ַ/���ݵĳ���
	{
		if(dat & 0x80) 		
			FT6336_SDA_H();		//����"1"		
		else
			FT6336_SDA_L();		//����"0"
		
		delay_us(10);
		FT6336_SCL_H();			//ʹSDA�ϵ�������Ч
		delay_us(10);			//SCL��С�ߵ�ƽ����:0.6us							
		FT6336_SCL_L();			//SCL��С�͵�ƽ����:1.2us
		delay_us(10);
		dat <<= 1;				//������������1λ,Ϊ��λ����׼��	
	}
}

/****************************************************
* �������� ��
* ��    �� ����Ƭ����IIC���߽���1B������
* ��ڲ��� ����
* ���ز��� ���յ���1B����
* ע������ ������һ�����������ݽ��չ���;�Ӹߵ��͵�˳���������
*****************************************************/
u8 FT6336_RdOneByte(void)						
{
	u8 i,dat = 0;				//��������λ���������ݴ浥Ԫ

	SDA_IN();						//SDA����Ϊ����
	for(i = 0;i < 8;i++)
	{
		FT6336_SCL_L();
		delay_us(10);
		FT6336_SCL_H();
		delay_us(10);			//SCL��С�͵�ƽ����:1.2us
		dat <<= 1;
		if(Is_SDA_IN != 0)
			dat |= 0x01;
		delay_us(5);			//SCL��С�͵�ƽ����:1.2us
	}
//	delay_us(2);
//	FT6336_SDA_H();		
	return(dat);				//����1B������
	
}
/****************************************************
* ��FT6336д��һ������
* reg:��ʼ�Ĵ�����ַ
* buf:���ݻ�������
* len:д���ݳ���
* ����ֵ:0,�ɹ�;1,ʧ��.
*****************************************************/
u8 FT6336_WR_Reg(u8 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	I2C_Start();	 
	FT6336_WrOneByte(FT_CMD_WR);	//����д���� 	 
	FT6336_CheckAck(); 	 										  		   
	FT6336_WrOneByte(reg);   	//���͵�8λ��ַ
	FT6336_CheckAck();  
	for(i=0;i<len;i++)
	{	   
		FT6336_WrOneByte(buf[i]);  	//������
		ret=FT6336_CheckAck();
		if(ret)break;  
	}
    I2C_End();					//����һ��ֹͣ����	    
	return ret; 
	

}
/*****************************************************
* ��FT6336����һ������
* reg:��ʼ�Ĵ�����ַ
* buf:���ݻ�������
* len:�����ݳ���
*****************************************************/
void FT6336_RD_Reg(u8 reg,u8 *buf,u8 len)
{
	u8 i; 
 	I2C_Start();	
 	FT6336_WrOneByte(FT_CMD_WR);   	//����д���� 	 
	FT6336_CheckAck(); 	 										  		   
 	FT6336_WrOneByte(reg);   	//���͵�8λ��ַ
	FT6336_CheckAck();  
//  I2C_End();					//����һ��ֹͣ����	 
 	I2C_Start();  	 	   
	FT6336_WrOneByte(FT_CMD_RD);   	//���Ͷ�����		   
	FT6336_CheckAck();	  
	for(i=0;i<(len-1);i++)
	{	   
		buf[i] = FT6336_RdOneByte();		//����1B���ݵ��������ݻ�������
		FT6336_McuACK();					//����Ӧ��λ	  
	}
	buf[i]  = FT6336_RdOneByte();	
	FT6336_McuNACK();						//n���ֽڶ���,���ͷ�Ӧ��λ
  I2C_End();					//����һ��ֹͣ����	  

} 
 



void FT6336U_start(void)
{
	SDA_OUT();     //sda???
	FT6336_SDA_H();	  	  
	FT6336_SCL_H();
	delay_us(10);   //8???,5us??,16???,???????,????8us,????????10us
 	FT6336_SDA_L();//START:when i2c is high,DATA change form high to low 
	delay_us(10);
	FT6336_SCL_L();//??I2C??,????????? 
}

void FT6336U_end(void)
{
	SDA_OUT();//sda???
	FT6336_SCL_L();
	FT6336_SDA_L();//STOP:when i2c is high DATA change form low to high
 	delay_us(10);
	FT6336_SCL_H(); 
	FT6336_SDA_H();//??I2C??????
	delay_us(10);							   	
}

void i2c_write_byte(unsigned char txd)
{			   	
	u8 t,i2c_sda;   
	SDA_OUT(); 	    
	FT6336_SCL_L();//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
//		i2c_sda=(txd&0x80)>>7;
 
		if(txd&0x80)
			FT6336_SDA_H();
		else 
			FT6336_SDA_L();
		
		txd<<=1; 	
		
		delay_us(10);   //��TEA5767��������ʱ���Ǳ����
		FT6336_SCL_H();
		delay_us(10); 
		FT6336_SCL_L();	
		delay_us(10);
	}	      
	FT6336_SDA_H();
	delay_us(10);
	SDA_IN();	   
	FT6336_SCL_H();
	delay_us(10);			 
	//	while(Is_SDA_IN);
	for(t=0;t<100;t++)//��ʱ100us�ȴ�
	{
		if(Is_SDA_IN != 0)
			delay_us(1);
		else
			break;
	}
	FT6336_SCL_L();

	//----------------------------������ʾ--------------------
	if(t >= 100)
	//string_normal(DIS_COL_BOUNDARY+1,UP_START+1,Red,"I2C_NCPL ERROR!");
	;
}

unsigned char i2c_read_byte(void)    //MCU��Ӧ��
{
	unsigned char i,receive=0;
	
	SDA_IN();//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		FT6336_SCL_L(); 
		delay_us(10);
		FT6336_SCL_H();
		receive<<=1;
		if(Is_SDA_IN != 0) 
			receive |= 0x01;   
		delay_us(10); 
	}					 

	FT6336_SCL_L();
	SDA_OUT();
	FT6336_SDA_H();
	delay_us(10);
	FT6336_SCL_H();
	delay_us(10);
	FT6336_SCL_L();

	FT6336_SDA_H();  
	return receive;

}

unsigned char i2c_read_keep_byte(void)    //MCU��Ӧ��
{
	unsigned char i,receive=0;
	
	SDA_IN();//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		FT6336_SCL_L(); 
		delay_us(10);
		FT6336_SCL_H();
		receive<<=1;
		if(Is_SDA_IN != 0) 
			receive |= 0x01;   
		delay_us(10); 
	}					 

	FT6336_SCL_L();
	SDA_OUT();
	FT6336_SDA_L();
	delay_us(10);
	FT6336_SCL_H();
	delay_us(10);
	FT6336_SCL_L();

	FT6336_SDA_H();  
	return receive;
}

unsigned char i2c_read_addr_byte(unsigned char device_addr,unsigned char read_addr)
{
	unsigned char dat;
	FT6336U_start();
	i2c_write_byte(device_addr<<1);
	i2c_write_byte(read_addr);
	FT6336U_end();

	FT6336U_start();
	i2c_write_byte((device_addr<<1) | 0x01);
	dat=i2c_read_byte();
	FT6336U_end();
	return(dat);
}

void i2c_read_addr_str(unsigned char device_addr,unsigned char read_addr,unsigned char read_amount,unsigned char *read_buf)
{
//	uchar dat;
	unsigned char i;
	FT6336U_start();
	i2c_write_byte(device_addr<<1);
	i2c_write_byte(read_addr);
	FT6336U_end();

	FT6336U_start();
	i2c_write_byte((device_addr<<1) | 0x01);

	for(i=0;i<read_amount-1;i++)
	{
		read_buf[i] = i2c_read_keep_byte();	
	}
	read_buf[i] = i2c_read_byte();
	FT6336U_end();
}


void i2c_write_addr_byte(unsigned char device_addr,unsigned char write_addr,unsigned char write_dat)
{
	FT6336U_start();
	i2c_write_byte(device_addr<<1);
	i2c_write_byte(write_addr);
	i2c_write_byte(write_dat);
	FT6336U_end();
	HAL_Delay(2);
}

unsigned int i2c_read_addr_int(unsigned char device_addr,unsigned char read_addr)
{
	unsigned char read_buf[2];
	i2c_read_addr_str(device_addr,read_addr,2,read_buf);
	return (read_buf[0]<<8)|read_buf[1];
}

void i2c_write_addr_str(unsigned char device_addr,unsigned char write_addr,unsigned char write_amount,unsigned char *write_buf)
{
	unsigned char i;
	FT6336U_start();
	i2c_write_byte(device_addr<<1);
	i2c_write_byte(write_addr);
	for(i=0;i<write_amount;i++)
	{
		i2c_write_byte(write_buf[i]);
	}
	FT6336U_end();
	HAL_Delay(2);
}

u8 FT6336_read_firmware_id(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_FIRMARE_ID);
}

//	u8 dat=0;
//
//	i2c_start();
//	i2c_write_byte((I2C_ADDR_FT6336<<1)|I2C_WR);
//	i2c_write_byte(0xA6);
//	i2c_end();
//
//	i2c_start();
//	i2c_write_byte((I2C_ADDR_FT6336<<1)|I2C_RE);
//	dat=i2c_read_byte();
//	i2c_end();
//	return(dat);

u8 FT6336_read_device_mode(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_DEVICE_MODE);
}

u8 FT6336_read_td_status(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_TD_STATUS);
}

//��һ����
u8 FT6336_read_touch1_event(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH1_EVENT)>>4;
}

u8 FT6336_read_touch1_id(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH1_ID)>>4;
}

//���½�Ϊ�������,ˮƽY��,��ֱX��
//Y�᷶Χ0~459
//X�᷶Χ0~319
//touch1��touch2����˳��Ϊ���������,��ID��Ϊ���µ�ʱ���Ⱥ���,�ñ�ſɴ����µ���ָ
//�ڵ�һ�㰴�º�ID��Ϊ0,�¼�Ϊ8,��̧���ID�����걣�������ֵ,�¼�Ϊ4,��ʱ�ڶ���������һֱΪFF
//�ڵ�һ�㰴��ǰ���°��ڶ�����,ID��Ϊ1,�¼�Ϊ8,̧���ȫ�ָ�FF,�����ڶ�����ĽӴ�������ڵ�һ����ʱ,
//��������ID������ǰ�������һ��������洢��ַ,��һ����̧���,�ڶ����������IDΪ1
//���µ�������ʱ,���������������������ԱȺ�,ֱ�����������С�ĵ�
u16 FT6336_read_touch1_x(void)
{
	u8 read_buf[2];
	i2c_read_addr_str(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH1_X,2,read_buf);
	return ((read_buf[0]&0x0f)<<8)|read_buf[1];
}

u16 FT6336_read_touch1_y(void)
{
	u8 read_buf[2];
	i2c_read_addr_str(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH1_Y,2,read_buf);
	return ((read_buf[0]&0x0f)<<8)|read_buf[1];
}

//�ڶ�����
u8 FT6336_read_touch2_event(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH2_EVENT)>>4;
}

u8 FT6336_read_touch2_id(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH2_ID)>>4;
}

u16 FT6336_read_touch2_x(void)
{
	u8 read_buf[2];
	i2c_read_addr_str(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH2_X,2,read_buf);
	return ((read_buf[0]&0x0f)<<8)|read_buf[1];
}

u16 FT6336_read_touch2_y(void)
{
	u8 read_buf[2];
	i2c_read_addr_str(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH2_Y,2,read_buf);
	return ((read_buf[0]&0x0f)<<8)|read_buf[1];
}


const u16 FT6236_TPX_TBL[5]=
{
	FT_TP1_REG,
	FT_TP2_REG,
	FT_TP3_REG,
	FT_TP4_REG,
	FT_TP5_REG
};

//����ʾ�⣨FPC���£�
////y��////////    //264x176
							//
							//
							//x��
							//
u8 sta = 0;							
u8 gestid=0;
u8 touch_count=0;
u8 buf[4] = {0}; 
u8 p1_xh = 0;
u8 p1_xl = 0;
u8 p1_yh = 0;
u8 p1_yl = 0;
u16 x_point = 0;
u16 y_point = 0;
TouchPointRefTypeDef TPR_Structure; 
void FT6336_Scan(void)
{
	u8 i=0;
	
	
	
	FT6336_RD_Reg(0x02,&sta,1);//��ȡ�������״̬  
  touch_count=sta;	
	FT6336_RD_Reg(0x01,&gestid,1);//��ȡ�������״̬
//	FT6336_RD_Reg(FT6236_TPX_TBL[1],buf,4);	//��ȡXY����ֵ
	if(sta & 0x0f)	//�ж��Ƿ��д����㰴�£�0x02�Ĵ����ĵ�4λ��ʾ��Ч�������
 	{
		 FT6336_RD_Reg(0x03,&p1_xh,1);
		 FT6336_RD_Reg(0x04,&p1_xl,1);
		 FT6336_RD_Reg(0x05,&p1_yh,1);
		 FT6336_RD_Reg(0x06,&p1_yl,1);
		 x_point =  ((p1_xh&0X07)<<8)|p1_xl;
		 y_point = ((p1_yh&0X07)<<8)|p1_yl;
		
	}
// 	if(sta & 0x0f)	//�ж��Ƿ��д����㰴�£�0x02�Ĵ����ĵ�4λ��ʾ��Ч�������
// 	{
// 		TPR_Structure.TouchSta = ~(0xFF << (sta & 0x0F));	//~(0xFF << (sta & 0x0F))����ĸ���ת��Ϊ�����㰴����Ч��־
// 		for(i=0;i<2;i++)	                                //�ֱ��жϴ�����1-5�Ƿ񱻰���
// 		{
// 			if(TPR_Structure.TouchSta & (1<<i))			    //��ȡ����������
// 			{											    //���������ȡ��Ӧ��������������
// 				FT6336_RD_Reg(FT6236_TPX_TBL[i],buf,4);	//��ȡXY����ֵ
//				TPR_Structure.x[i]=((u16)(buf[0]&0X0F)<<8)+buf[1];
//				TPR_Structure.y[i]=((u16)(buf[2]&0X0F)<<8)+buf[3];
// 				if((buf[0]&0XC0)!=0X80)
// 				{
//					TPR_Structure.x[i]=TPR_Structure.y[i]=0;//������contact�¼�������Ϊ��Ч	
//					TPR_Structure.TouchSta &=0xe0;	//�����������Ч���
//					return;
//				}
// 			}
// 		}
// 		TPR_Structure.TouchSta |= TP_PRES_DOWN;     //�������±��
// 	}
// 	else
// 	{
// 		if(TPR_Structure.TouchSta &TP_PRES_DOWN) 	//֮ǰ�Ǳ����µ�
// 			TPR_Structure.TouchSta &= ~0x80;        //�����ɿ����	
// 		else
// 		{
// 			TPR_Structure.x[0] = 0;
// 			TPR_Structure.y[0] = 0;
// 			TPR_Structure.TouchSta &=0xe0;	//�����������Ч���
// 		}
// 	}
}

