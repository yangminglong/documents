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
**函数名：FT6236_Init
**传入参数：无
**返回值：无
**功能：初始化FT6236引脚
*/  
u8 id = 0;
u8 panel_id = 0;
u8 dev_ode = 0;
u8 code_ode = 0;
  void FT6336_Init(void)
{
	u8 temp,KK;
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOC_CLK_ENABLE();   //使能GPIOB时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();   //使能GPIOB时钟
    
    //初始化设置
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;     //快速
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
	FT6336_WR_Reg(FT_DEVIDE_MODE,&temp,1);	//进入正常操作模式 
	FT6336_RD_Reg(FT_DEVIDE_MODE,&dev_ode,1);	
	
		FT6336_RD_Reg(0xA3,&id,1);	
		FT6336_RD_Reg(0xA8,&panel_id,1);	
		FT6336_RD_Reg(0xAf,&code_ode,1);	
		
    
		
		
 	temp=22;								//触摸有效值，22，越小越灵敏	
 	FT6336_WR_Reg(FT_ID_G_THGROUP,&temp,1);	//设置触摸有效值
	FT6336_RD_Reg(FT_ID_G_THGROUP,&KK,1);
 	temp=14;								//激活周期，不能小于12，最大14
 	FT6336_WR_Reg(FT_ID_G_PERIODACTIVE,&temp,1); 
	FT6336_RD_Reg(FT_ID_G_PERIODACTIVE,&KK,1);
/******************************************************/
}




//-------------------------------分割复制部分-----------------------
/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送起始信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：
*****************************************************/
void I2C_Start(void)
{
	SDA_OUT();     //sda线输出
	FT6336_SDA_H();	  	  
	FT6336_SCL_H();
	delay_us(10);   //8通道时,5us即可,16通道时,由于拉电流下降,至少需要8us,为了保证没问题设10us
 	FT6336_SDA_L();//START:when i2c is high,DATA change form high to low 
	delay_us(5);
	FT6336_SCL_L();//钳住I2C总线，准备发送或接收数据 
	delay_us(10);
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送停止信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：
*****************************************************/
void I2C_End(void)
{
//	SDA_OUT();     		//sda线输出
//	FT6336_SDA_L();		
//	delay_us(5);	
//	FT6336_SCL_H();		//SCL最小高电平脉宽:0.6us		
////	delay_us(4);		//停止信号的最小建立时间:0.6us
////	FT6336_SDA_L();	
//	delay_us(10);
//	FT6336_SDA_H();		//SCL高电平期间，SDA的一个上升沿表示停止信号
//	delay_us(5);		
	
	SDA_OUT();     		//sda线输出
	FT6336_SCL_L();		//SCL最小高电平脉宽:0.6us		
	FT6336_SDA_L();		
	delay_us(5);	
	FT6336_SCL_H();		//SCL最小高电平脉宽:0.6us		
	delay_us(4);		//停止信号的最小建立时间:0.6us
	FT6336_SDA_H();		//SCL高电平期间，SDA的一个上升沿表示停止信号
	delay_us(5);	
}


/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送应答信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：单片机读1B数据后发送一个应答信号
*****************************************************/
void FT6336_McuACK(void)							
{
	FT6336_SCL_L();
	SDA_OUT();     		//sda线输出	
	FT6336_SDA_L();
	delay_us(5);																	
	FT6336_SCL_H();		//SCL最小高电平脉宽:0.6us
	delay_us(10);
	FT6336_SCL_L();	//SCL最小低电平脉宽:1.2us
	delay_us(10);
	
	FT6336_SDA_H();
}


/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送非应答信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：单片机读数据停止前发送一个非应答信号
*****************************************************/
void FT6336_McuNACK(void)
{
	FT6336_SCL_L();
	SDA_OUT();     				//sda线输出	
	FT6336_SDA_H();
	delay_us(5);																	
	FT6336_SCL_H();				//SCL最小高电平脉宽:0.6us
	delay_us(10);
	FT6336_SCL_L();			//SCL最小低电平脉宽:1.2us
	delay_us(10);
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机检查FT6236送来的应答信号
* 入口参数 ：无
* 返回参数 ：1，接收应答失败
			 0，接收应答成功
* 注意事项 ：单片机写1个地址/数据后检查
			 全局变量RevAckF:收到FT6236应答信号的标志位,为0表示收到
*****************************************************/
u8 FT6336_CheckAck(void)							
{
	u8 ucErrTime=0;
	u8 redata;
	
	FT6336_SDA_H();
//	delay_us(5);  
	SDA_IN();  //SDA设置为输入
	FT6336_SCL_H();			//使SDA上数据有效;SCL最小高电平脉宽:0.6us
	delay_us(10);
	while(Is_SDA_IN != 0)
	{	
		ucErrTime++;
		if(ucErrTime>250)		//无应答
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
* 函数名称 ：
* 功    能 ：单片机向IIC总线发送1B的地址/数据
* 入口参数 ：待发送的1B地址/数据
* 返回参数 ：无
* 注意事项 ：不是一个完整的数据发送过程;送数的顺序是从高到低
*****************************************************/
void FT6336_WrOneByte(u8 dat)						
{
	u8 i;						
	SDA_OUT();     				//sda线输出	
	FT6336_SCL_L();				//拉低时钟开始数据传输
	for(i = 0; i < 8; i++)		//8位1B地址/数据的长度
	{
		if(dat & 0x80) 		
			FT6336_SDA_H();		//发送"1"		
		else
			FT6336_SDA_L();		//发送"0"
		
		delay_us(10);
		FT6336_SCL_H();			//使SDA上的数据有效
		delay_us(10);			//SCL最小高电平脉宽:0.6us							
		FT6336_SCL_L();			//SCL最小低电平脉宽:1.2us
		delay_us(10);
		dat <<= 1;				//发送数据左移1位,为下位发送准备	
	}
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机从IIC总线接收1B的数据
* 入口参数 ：无
* 返回参数 ：收到的1B数据
* 注意事项 ：不是一个完整的数据接收过程;从高到低的顺序接收数据
*****************************************************/
u8 FT6336_RdOneByte(void)						
{
	u8 i,dat = 0;				//接收数据位数和内容暂存单元

	SDA_IN();						//SDA设置为输入
	for(i = 0;i < 8;i++)
	{
		FT6336_SCL_L();
		delay_us(10);
		FT6336_SCL_H();
		delay_us(10);			//SCL最小低电平脉宽:1.2us
		dat <<= 1;
		if(Is_SDA_IN != 0)
			dat |= 0x01;
		delay_us(5);			//SCL最小低电平脉宽:1.2us
	}
//	delay_us(2);
//	FT6336_SDA_H();		
	return(dat);				//返回1B的数据
	
}
/****************************************************
* 向FT6336写入一次数据
* reg:起始寄存器地址
* buf:数据缓缓存区
* len:写数据长度
* 返回值:0,成功;1,失败.
*****************************************************/
u8 FT6336_WR_Reg(u8 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	I2C_Start();	 
	FT6336_WrOneByte(FT_CMD_WR);	//发送写命令 	 
	FT6336_CheckAck(); 	 										  		   
	FT6336_WrOneByte(reg);   	//发送低8位地址
	FT6336_CheckAck();  
	for(i=0;i<len;i++)
	{	   
		FT6336_WrOneByte(buf[i]);  	//发数据
		ret=FT6336_CheckAck();
		if(ret)break;  
	}
    I2C_End();					//产生一个停止条件	    
	return ret; 
	

}
/*****************************************************
* 从FT6336读出一次数据
* reg:起始寄存器地址
* buf:数据缓缓存区
* len:读数据长度
*****************************************************/
void FT6336_RD_Reg(u8 reg,u8 *buf,u8 len)
{
	u8 i; 
 	I2C_Start();	
 	FT6336_WrOneByte(FT_CMD_WR);   	//发送写命令 	 
	FT6336_CheckAck(); 	 										  		   
 	FT6336_WrOneByte(reg);   	//发送低8位地址
	FT6336_CheckAck();  
//  I2C_End();					//产生一个停止条件	 
 	I2C_Start();  	 	   
	FT6336_WrOneByte(FT_CMD_RD);   	//发送读命令		   
	FT6336_CheckAck();	  
	for(i=0;i<(len-1);i++)
	{	   
		buf[i] = FT6336_RdOneByte();		//读入1B数据到接收数据缓冲区中
		FT6336_McuACK();					//发送应答位	  
	}
	buf[i]  = FT6336_RdOneByte();	
	FT6336_McuNACK();						//n个字节读完,发送非应答位
  I2C_End();					//产生一个停止条件	  

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
	FT6336_SCL_L();//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
//		i2c_sda=(txd&0x80)>>7;
 
		if(txd&0x80)
			FT6336_SDA_H();
		else 
			FT6336_SDA_L();
		
		txd<<=1; 	
		
		delay_us(10);   //对TEA5767这三个延时都是必须的
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
	for(t=0;t<100;t++)//延时100us等待
	{
		if(Is_SDA_IN != 0)
			delay_us(1);
		else
			break;
	}
	FT6336_SCL_L();

	//----------------------------错误提示--------------------
	if(t >= 100)
	//string_normal(DIS_COL_BOUNDARY+1,UP_START+1,Red,"I2C_NCPL ERROR!");
	;
}

unsigned char i2c_read_byte(void)    //MCU无应答
{
	unsigned char i,receive=0;
	
	SDA_IN();//SDA设置为输入
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

unsigned char i2c_read_keep_byte(void)    //MCU有应答
{
	unsigned char i,receive=0;
	
	SDA_IN();//SDA设置为输入
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

//第一触点
u8 FT6336_read_touch1_event(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH1_EVENT)>>4;
}

u8 FT6336_read_touch1_id(void)
{
	return i2c_read_addr_byte(I2C_ADDR_FT6336,FT6336_ADDR_TOUCH1_ID)>>4;
}

//左下角为坐标零点,水平Y轴,垂直X轴
//Y轴范围0~459
//X轴范围0~319
//touch1和touch2排列顺序为按面积排列,而ID号为按下的时间先后标号,该标号可代表按下的手指
//在第一点按下后ID记为0,事件为8,而抬起后ID与坐标保持最后数值,事件为4,此时第二个点数据一直为FF
//在第一点按下前提下按第二个点,ID记为1,事件为8,抬起后全恢复FF,而当第二个点的接触面积大于第一个点时,
//它将保持ID与坐标前提下与第一个点更换存储地址,第一个点抬起后,第二个点的依旧ID为1
//按下第三个点时,将与其它两个点进行面积对比后,直接抛弃面积最小的点
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

//第二触点
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

//坐标示意（FPC朝下）
////y轴////////    //264x176
							//
							//
							//x轴
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
	
	
	
	FT6336_RD_Reg(0x02,&sta,1);//读取触摸点的状态  
  touch_count=sta;	
	FT6336_RD_Reg(0x01,&gestid,1);//读取触摸点的状态
//	FT6336_RD_Reg(FT6236_TPX_TBL[1],buf,4);	//读取XY坐标值
	if(sta & 0x0f)	//判断是否有触摸点按下，0x02寄存器的低4位表示有效触点个数
 	{
		 FT6336_RD_Reg(0x03,&p1_xh,1);
		 FT6336_RD_Reg(0x04,&p1_xl,1);
		 FT6336_RD_Reg(0x05,&p1_yh,1);
		 FT6336_RD_Reg(0x06,&p1_yl,1);
		 x_point =  ((p1_xh&0X07)<<8)|p1_xl;
		 y_point = ((p1_yh&0X07)<<8)|p1_yl;
		
	}
// 	if(sta & 0x0f)	//判断是否有触摸点按下，0x02寄存器的低4位表示有效触点个数
// 	{
// 		TPR_Structure.TouchSta = ~(0xFF << (sta & 0x0F));	//~(0xFF << (sta & 0x0F))将点的个数转换为触摸点按下有效标志
// 		for(i=0;i<2;i++)	                                //分别判断触摸点1-5是否被按下
// 		{
// 			if(TPR_Structure.TouchSta & (1<<i))			    //读取触摸点坐标
// 			{											    //被按下则读取对应触摸点坐标数据
// 				FT6336_RD_Reg(FT6236_TPX_TBL[i],buf,4);	//读取XY坐标值
//				TPR_Structure.x[i]=((u16)(buf[0]&0X0F)<<8)+buf[1];
//				TPR_Structure.y[i]=((u16)(buf[2]&0X0F)<<8)+buf[3];
// 				if((buf[0]&0XC0)!=0X80)
// 				{
//					TPR_Structure.x[i]=TPR_Structure.y[i]=0;//必须是contact事件，才认为有效	
//					TPR_Structure.TouchSta &=0xe0;	//清除触摸点有效标记
//					return;
//				}
// 			}
// 		}
// 		TPR_Structure.TouchSta |= TP_PRES_DOWN;     //触摸按下标记
// 	}
// 	else
// 	{
// 		if(TPR_Structure.TouchSta &TP_PRES_DOWN) 	//之前是被按下的
// 			TPR_Structure.TouchSta &= ~0x80;        //触摸松开标记	
// 		else
// 		{
// 			TPR_Structure.x[0] = 0;
// 			TPR_Structure.y[0] = 0;
// 			TPR_Structure.TouchSta &=0xe0;	//清除触摸点有效标记
// 		}
// 	}
}

