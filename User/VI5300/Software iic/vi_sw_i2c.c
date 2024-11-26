/**
  ******************************************************************************
  * @file    vi_sw_i2c.c
  * @author  cxg
  * @version V1.0
  * @date    2021-01-01
  * @brief   软件IIC 驱动
  ******************************************************************************
  
  */

/*
	应用说明：
	在访问I2C设备前，请先调用 i2c_CheckDevice() 检测I2C设备是否正常，该函数会配置GPIO
*/
#ifdef USE_SOFTWARE_IIC

#include "vi_sw_i2c.h"

//#define QMAX981_SW_I2C_ADDR_W		0x24
//#define QMAX981_SW_I2C_ADDR_R		0x25
/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面4行代码即可任意改变SCL和SDA的引脚 */
#define GPIO_PORT_I2C	GPIOD			/* GPIO端口 */
//#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define I2C_SCL_PIN		GPIO_PIN_6	//	GPIO_Pin_4	/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_PIN_7  //	GPIO_Pin_1		/* 连接到SDA数据线的GPIO */




/* 定义读写SCL和SDA的宏，已增加代码的可移植性和可阅读性 */
	#define I2C_SCL_OUTPUT()			
	#define I2C_SCL_INPUT()			
	#define I2C_SDA_OUTPUT()			
	#define I2C_SDA_INPUT()			
#if 0	/* 条件编译： 1 选择GPIO的库函数实现IO读写 */
#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_PORT_I2C->BRR = I2C_SCL_PIN				/* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_PORT_I2C->BRR = I2C_SDA_PIN				/* SDA = 0 */
	
	#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */


#else	/* 这个分支选择直接寄存器操作实现IO读写 */
    /*　注意：如下写法，在IAR最高级别优化时，会被编译器错误优化 */
	#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN				/* SCL = 1 */
	#define I2C_SCL_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SCL_PIN << 16U /* SCL = 0 */
	
	#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN				/* SDA = 1 */
	#define I2C_SDA_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SDA_PIN << 16U /* SDA = 0 */
	
	#define I2C_SDA_READ()  ((GPIO_PORT_I2C->IDR & I2C_SDA_PIN) != 0)	/* 读SDA口线状态 */
#endif


void i2c_SW_GPIO_Config(void);
void i2c_Ack(void);
void i2c_NAck(void);

/*
*********************************************************************************************************
*	函 数 名: i2c_Delay
*	功能说明: I2C总线位延迟，最快400KHz
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	uint8_t i;

	/*　
	 	下面的时间是通过安富莱AX-Pro逻辑分析仪测试得到的。
		CPU主频72MHz时，在内部Flash运行, MDK工程不优化
		循环次数为10时，SCL频率 = 205KHz 
		循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us 
	 	循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us 
        
    IAR工程编译效率高，不能设置为7
	*/
	for (i = 0; i < 23; i++);
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();
	
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	I2C_SCL_OUTPUT();
	I2C_SDA_OUTPUT();

	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参：_ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{		
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();	
		I2C_SCL_0();
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */
		i2c_Delay();
	}
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参：无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(unsigned char ack)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	I2C_SDA_INPUT();	// set data input	
	i2c_Delay();
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		//I2C_SCL_1();
		//i2c_Delay();
		I2C_SCL_0();
		i2c_Delay();
	}
	
	I2C_SDA_OUTPUT();	// set data output	
	i2c_Delay();
	if(ack==0)
		i2c_NAck();
	else
		i2c_Ack();
	return value;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	I2C_SDA_INPUT();	//set data input
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	I2C_SDA_OUTPUT();	//set data input
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*
*********************************************************************************************************
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();	
}

/*
*********************************************************************************************************
*	函 数 名: i2c_sw_gpio_config
*	功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_sw_gpio_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* 打开GPIO时钟 */
	GPIO_InitStructure.Pin = I2C_SCL_PIN |I2C_SDA_PIN ;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode =GPIO_MODE_OUTPUT_OD;/* 开漏输出 */
   HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);


	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	//i2c_Stop();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参：_Address：设备的I2C总线地址
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
uint8_t i2c_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	i2c_sw_gpio_config();		/* 配置GPIO */
	
	i2c_Start();		/* 发送启动信号 */

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(_Address|I2C_WR);
	ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */

	i2c_Stop();			/* 发送停止信号 */

	return ucAck;
}

uint8_t vi_sw_writereg(uint8_t slave, uint8_t reg_add,uint8_t reg_dat)
{
	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_dat);	
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_Stop();

	return 1;
}

uint8_t vi_sw_readreg(uint8_t slave, uint8_t reg_add,uint8_t *buf,uint8_t num)
{
	//uint8_t ret;
	uint8_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	i2c_SendByte(reg_add);
	if(i2c_WaitAck())
	{
		return 0;
	}

	i2c_Start();
	i2c_SendByte(slave|0x01);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(num-1);i++){
		*buf=i2c_ReadByte(1);
		buf++;
	}
	*buf=i2c_ReadByte(0);
	i2c_Stop();

	return 1;
}

uint8_t vi_iic_writeRegs(uint8_t slave, uint8_t* data, uint8_t Len)
{
	uint8_t i;

	i2c_Start();
	i2c_SendByte(slave);
	if(i2c_WaitAck())
	{
		return 0;
	}
	/*
	i2c_SendByte(Addr);	
	if(i2c_WaitAck())
	{
		return 0;
	}*/
	for(i=0;i<Len;i++)
	{
		i2c_SendByte(data[i]);	
		if(i2c_WaitAck())
		{
			return 0;
		}
	}
	i2c_Stop();

	return 1;
}


uint8_t vi_iic_readRegs(uint8_t slave, uint8_t *data, uint16_t Length)
{
	uint8_t i;

	i2c_Start();
	i2c_SendByte(slave|1);
	if(i2c_WaitAck())
	{
		return 0;
	}

	for(i=0;i<(Length-1);i++){
		*data=i2c_ReadByte(1);
		data++;
	}
	*data=i2c_ReadByte(0);
	i2c_Stop();

	return 1;

}
#endif
