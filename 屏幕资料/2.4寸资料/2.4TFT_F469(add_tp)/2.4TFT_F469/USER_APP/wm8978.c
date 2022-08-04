#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gpio.h"
#include "sys.h"
#include "wm8978.h"

//void delay_us(uint32_t us) 
//{
//    uint32_t i = 0;
//    for(i=0; i<us; i++) {
//        uint8_t a = 2;
//        while(a--);
//    }
//}

//IIC初始化
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //使能GPIOB时钟
    
    //PH4,5初始化设置
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;     //快速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    IIC_SDA=1;
    IIC_SCL=1;  
	
	  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_Delay(10);
	
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}
//WM8978寄存器值缓存区(总共58个寄存器,0~57),占用116字节内存
//因为WM8978的IIC操作不支持读操作,所以在本地保存所有寄存器值
//写WM8978寄存器时,同步更新到本地寄存器值,读寄存器时,直接返回本地保存的寄存器值.
//注意:WM8978的寄存器值是9位的,所以要用u16来存储. 
static u16 WM8978_REGVAL_TBL[58]=
{
	0X0000,0X0000,0X0000,0X0000,0X0050,0X0000,0X0140,0X0000,
	0X0000,0X0000,0X0000,0X00FF,0X00FF,0X0000,0X0100,0X00FF,
	0X00FF,0X0000,0X012C,0X002C,0X002C,0X002C,0X002C,0X0000,
	0X0032,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
	0X0038,0X000B,0X0032,0X0000,0X0008,0X000C,0X0093,0X00E9,
	0X0000,0X0000,0X0000,0X0000,0X0003,0X0010,0X0010,0X0100,
	0X0100,0X0002,0X0001,0X0001,0X0039,0X0039,0X0039,0X0039,
	0X0001,0X0001
}; 
#define WM8978_ADDR     0X1A	//WM8978的器件地址,固定为0X1A 
//WM8978写寄存器
//reg:寄存器地址
//val:要写入寄存器的值 
//返回值:0,成功;
//    其他,错误代码
u8 WM8978_Write_Reg(u8 reg,u16 val)
{ 
	IIC_Start(); 
	IIC_Send_Byte((0x38<<1)|0);//发送器件地址+写命令	 
	if(IIC_Wait_Ack())return 1;	//等待应答(成功?/失败?) 
  IIC_Send_Byte((reg<<1)|((val>>8)&0X01));//写寄存器地址+数据的最高位
	
	if(IIC_Wait_Ack())return 2;	//等待应答(成功?/失败?) 
	IIC_Send_Byte(val&0XFF);	//发送数据
	
	if(IIC_Wait_Ack())return 3;	//等待应答(成功?/失败?) 
  IIC_Stop();
	WM8978_REGVAL_TBL[reg]=val;	//保存寄存器值到本地
	return 0;	
} 


//WM8978初始化
//返回值:0,初始化正常
//    其他,错误代码
u8 WM8978_Init(void)
{ 
	u8 res;
	
	IIC_Init();                 //初始化IIC接口
	res=WM8978_Write_Reg(0,0);	//软复位WM8978
	if(res)return 1;			//发送指令失败,WM8978异常
	//以下为通用设置
	WM8978_Write_Reg(1,0X1B);	//R1,MICEN设置为1(MIC使能),BIASEN设置为1(模拟器工作),VMIDSEL[1:0]设置为:11(5K)
	WM8978_Write_Reg(2,0X1B0);	//R2,ROUT1,LOUT1输出使能(耳机可以工作),BOOSTENR,BOOSTENL使能
	WM8978_Write_Reg(3,0X6C);	//R3,LOUT2,ROUT2输出使能(喇叭工作),RMIX,LMIX使能	
	WM8978_Write_Reg(6,0);		//R6,MCLK由外部提供
	WM8978_Write_Reg(43,1<<4);	//R43,INVROUT2反向,驱动喇叭
	WM8978_Write_Reg(47,1<<8);	//R47设置,PGABOOSTL,左通道MIC获得20倍增益
	WM8978_Write_Reg(48,1<<8);	//R48设置,PGABOOSTR,右通道MIC获得20倍增益
	WM8978_Write_Reg(49,1<<1);	//R49,TSDEN,开启过热保护 
	WM8978_Write_Reg(49,1<<2);	//R49,SPEAKER BOOST,1.5x 
	WM8978_Write_Reg(10,1<<3);	//R10,SOFTMUTE关闭,128x采样,最佳SNR 
	WM8978_Write_Reg(14,1<<3);	//R14,ADC 128x采样率
	return 0;
} 

 
//WM8978读寄存器
//就是读取本地寄存器值缓冲区内的对应值
//reg:寄存器地址 
//返回值:寄存器值
u16 WM8978_Read_Reg(u8 reg)
{  
	return WM8978_REGVAL_TBL[reg];	
} 
//WM8978 DAC/ADC配置
//adcen:adc使能(1)/关闭(0)
//dacen:dac使能(1)/关闭(0)
void WM8978_ADDA_Cfg(u8 dacen,u8 adcen)
{
	u16 regval;
	regval=WM8978_Read_Reg(3);	//读取R3
	if(dacen)regval|=3<<0;		//R3最低2个位设置为1,开启DACR&DACL
	else regval&=~(3<<0);		//R3最低2个位清零,关闭DACR&DACL.
	WM8978_Write_Reg(3,regval);	//设置R3
	regval=WM8978_Read_Reg(2);	//读取R2
	if(adcen)regval|=3<<0;		//R2最低2个位设置为1,开启ADCR&ADCL
	else regval&=~(3<<0);		//R2最低2个位清零,关闭ADCR&ADCL.
	WM8978_Write_Reg(2,regval);	//设置R2	
}
//WM8978 输入通道配置 
//micen:MIC开启(1)/关闭(0)
//lineinen:Line In开启(1)/关闭(0)
//auxen:aux开启(1)/关闭(0) 
void WM8978_Input_Cfg(u8 micen,u8 lineinen,u8 auxen)
{
	u16 regval;  
	regval=WM8978_Read_Reg(2);	//读取R2
	if(micen)regval|=3<<2;		//开启INPPGAENR,INPPGAENL(MIC的PGA放大)
	else regval&=~(3<<2);		//关闭INPPGAENR,INPPGAENL.
 	WM8978_Write_Reg(2,regval);	//设置R2 
	
	regval=WM8978_Read_Reg(44);	//读取R44
	if(micen)regval|=3<<4|3<<0;	//开启LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	else regval&=~(3<<4|3<<0);	//关闭LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	WM8978_Write_Reg(44,regval);//设置R44
	
	if(lineinen)WM8978_LINEIN_Gain(5);//LINE IN 0dB增益
	else WM8978_LINEIN_Gain(0);	//关闭LINE IN
	if(auxen)WM8978_AUX_Gain(7);//AUX 6dB增益
	else WM8978_AUX_Gain(0);	//关闭AUX输入  
}
//WM8978 输出配置 
//dacen:DAC输出(放音)开启(1)/关闭(0)
//bpsen:Bypass输出(录音,包括MIC,LINE IN,AUX等)开启(1)/关闭(0) 
void WM8978_Output_Cfg(u8 dacen,u8 bpsen)
{
	u16 regval=0;
	if(dacen)regval|=1<<0;	//DAC输出使能
	if(bpsen)
	{
		regval|=1<<1;		//BYPASS使能
		regval|=5<<2;		//0dB增益
	} 
	WM8978_Write_Reg(50,regval);//R50设置
	WM8978_Write_Reg(51,regval);//R51设置 
}
//WM8978 MIC增益设置(不包括BOOST的20dB,MIC-->ADC输入部分的增益)
//gain:0~63,对应-12dB~35.25dB,0.75dB/Step
void WM8978_MIC_Gain(u8 gain)
{
	gain&=0X3F;
	WM8978_Write_Reg(45,gain);		//R45,左通道PGA设置 
	WM8978_Write_Reg(46,gain|1<<8);	//R46,右通道PGA设置
}
//WM8978 L2/R2(也就是Line In)增益设置(L2/R2-->ADC输入部分的增益)
//gain:0~7,0表示通道禁止,1~7,对应-12dB~6dB,3dB/Step
void WM8978_LINEIN_Gain(u8 gain)
{
	u16 regval;
	gain&=0X07;
	regval=WM8978_Read_Reg(47);	//读取R47
	regval&=~(7<<4);			//清除原来的设置 
 	WM8978_Write_Reg(47,regval|gain<<4);//设置R47
	regval=WM8978_Read_Reg(48);	//读取R48
	regval&=~(7<<4);			//清除原来的设置 
 	WM8978_Write_Reg(48,regval|gain<<4);//设置R48
} 
//WM8978 AUXR,AUXL(PWM音频部分)增益设置(AUXR/L-->ADC输入部分的增益)
//gain:0~7,0表示通道禁止,1~7,对应-12dB~6dB,3dB/Step
void WM8978_AUX_Gain(u8 gain)
{
	u16 regval;
	gain&=0X07;
	regval=WM8978_Read_Reg(47);	//读取R47
	regval&=~(7<<0);			//清除原来的设置 
 	WM8978_Write_Reg(47,regval|gain<<0);//设置R47
	regval=WM8978_Read_Reg(48);	//读取R48
	regval&=~(7<<0);			//清除原来的设置 
 	WM8978_Write_Reg(48,regval|gain<<0);//设置R48
}  
//设置I2S工作模式
//fmt:0,LSB(右对齐);1,MSB(左对齐);2,飞利浦标准I2S;3,PCM/DSP;
//len:0,16位;1,20位;2,24位;3,32位;  
void WM8978_I2S_Cfg(u8 fmt,u8 len)
{
	fmt&=0X03;
	len&=0X03;//限定范围
	WM8978_Write_Reg(4,(fmt<<3)|(len<<5));	//R4,WM8978工作模式设置	
}	

//设置耳机左右声道音量
//voll:左声道音量(0~63)
//volr:右声道音量(0~63)
void WM8978_HPvol_Set(u8 voll,u8 volr)
{
	voll&=0X3F;
	volr&=0X3F;//限定范围
	if(voll==0)voll|=1<<6;//音量为0时,直接mute
	if(volr==0)volr|=1<<6;//音量为0时,直接mute 
	WM8978_Write_Reg(52,voll);			//R52,耳机左声道音量设置
	WM8978_Write_Reg(53,volr|(1<<8));	//R53,耳机右声道音量设置,同步更新(HPVU=1)
}
//设置喇叭音量
//voll:左声道音量(0~63) 
void WM8978_SPKvol_Set(u8 volx)
{ 
	volx&=0X3F;//限定范围
	if(volx==0)volx|=1<<6;//音量为0时,直接mute 
 	WM8978_Write_Reg(54,volx);			//R54,喇叭左声道音量设置
	WM8978_Write_Reg(55,volx|(1<<8));	//R55,喇叭右声道音量设置,同步更新(SPKVU=1)	
}
//设置3D环绕声
//depth:0~15(3D强度,0最弱,15最强)
void WM8978_3D_Set(u8 depth)
{ 
	depth&=0XF;//限定范围 
 	WM8978_Write_Reg(41,depth);	//R41,3D环绕设置 	
}
//设置EQ/3D作用方向
//dir:0,在ADC起作用
//    1,在DAC起作用(默认)
void WM8978_EQ_3D_Dir(u8 dir)
{
	u16 regval; 
	regval=WM8978_Read_Reg(0X12);
	if(dir)regval|=1<<8;
	else regval&=~(1<<8); 
 	WM8978_Write_Reg(18,regval);//R18,EQ1的第9位控制EQ/3D方向
}

//设置EQ1
//cfreq:截止频率,0~3,分别对应:80/105/135/175Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ1_Set(u8 cfreq,u8 gain)
{ 
	u16 regval;
	cfreq&=0X3;//限定范围 
	if(gain>24)gain=24;
	gain=24-gain;
	regval=WM8978_Read_Reg(18);
	regval&=0X100;
	regval|=cfreq<<5;	//设置截止频率 
	regval|=gain;		//设置增益	
 	WM8978_Write_Reg(18,regval);//R18,EQ1设置 	
}
//设置EQ2
//cfreq:中心频率,0~3,分别对应:230/300/385/500Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ2_Set(u8 cfreq,u8 gain)
{ 
	u16 regval=0;
	cfreq&=0X3;//限定范围 
	if(gain>24)gain=24;
	gain=24-gain; 
	regval|=cfreq<<5;	//设置截止频率 
	regval|=gain;		//设置增益	
 	WM8978_Write_Reg(19,regval);//R19,EQ2设置 	
}
//设置EQ3
//cfreq:中心频率,0~3,分别对应:650/850/1100/1400Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ3_Set(u8 cfreq,u8 gain)
{ 
	u16 regval=0;
	cfreq&=0X3;//限定范围 
	if(gain>24)gain=24;
	gain=24-gain; 
	regval|=cfreq<<5;	//设置截止频率 
	regval|=gain;		//设置增益	
 	WM8978_Write_Reg(20,regval);//R20,EQ3设置 	
}
//设置EQ4
//cfreq:中心频率,0~3,分别对应:1800/2400/3200/4100Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ4_Set(u8 cfreq,u8 gain)
{ 
	u16 regval=0;
	cfreq&=0X3;//限定范围 
	if(gain>24)gain=24;
	gain=24-gain; 
	regval|=cfreq<<5;	//设置截止频率 
	regval|=gain;		//设置增益	
 	WM8978_Write_Reg(21,regval);//R21,EQ4设置 	
}
//设置EQ5
//cfreq:中心频率,0~3,分别对应:5300/6900/9000/11700Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ5_Set(u8 cfreq,u8 gain)
{ 
	u16 regval=0;
	cfreq&=0X3;//限定范围 
	if(gain>24)gain=24;
	gain=24-gain; 
	regval|=cfreq<<5;	//设置截止频率 
	regval|=gain;		//设置增益	
 	WM8978_Write_Reg(22,regval);//R22,EQ5设置 	
}


#define	DEVICE_ADDRESS						0x1A
#define	WIRTE_ADDRESS							(DEVICE_ADDRESS << 1 | 0)

uint32_t WM8978_Reset(void)
{
	if(WM8978_Write_Reg(0, 0)!= HAL_OK)// 软复位
		return 1;
	HAL_Delay(10);
	return 0;
}
uint8_t ft_id;
uint32_t WM8978_In(void)
{
//  WM8978_Init();

//	WM8978_SPKvol_Set(50);			//喇叭音量设置
//	WM8978_ADDA_Cfg(1,0);	//开启DAC
//	WM8978_Input_Cfg(0,0,0);//关闭输入通道
//	WM8978_Output_Cfg(1,0);	//开启DAC输出  
//	
	IIC_Init();
//	WM8978_Write_Reg(0xA6, ft_id);
	
	for(uint8_t i=0;i<0xff;i++)
	{
		IIC_Start(); 
		IIC_Send_Byte((i)|0);//发送器件地址+写命令	 
		IIC_Wait_Ack();//等待应答(成功?/失败?) 
	}
//	if(WM8978_Write_Reg(0, 0)!= HAL_OK)// 软复位
//	  return 1;
//	WM8978_Reset();
//	WM8978_Write_Reg(1, 0x0F);	// 模拟放大器使能， 使能输出输入缓存区
//	WM8978_Write_Reg(3, 0x7F);	// 使能左右声道和L\ROUT2
//	WM8978_Write_Reg(4, 0x10);	// I2S 16bit
//	WM8978_Write_Reg(6, 0x00);	// MCU提供时钟
//	WM8978_Write_Reg(10, 0x08);	// 输出音质最好
//	WM8978_Write_Reg(43, 0x10);	// ROUT2反相
////	
//	 WM8978_SPKvol_Set(63);
////	WM8978_VolumeCtl(0x3F);
	
	
	return 0;
}

