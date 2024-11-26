#include "VI5300_API.h"
#include "VI5300_Config.h"
#include "VI5300_Firmware.h"
#include "string.h"

uint8_t gSalve=VI5300_DEVICE_ADDR,chip_reg=0,chip_reg38,chip_reg3a;

#define FIS_INT1_Pin        GPIO_PIN_3
#define FIS_INT1_GPIO_Port  GPIOB

void VI5300_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
#if 0
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = FIS_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL 会误触发中断
	HAL_GPIO_Init(FIS_INT1_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif
	//control  Xshut pin
	GPIO_InitStruct.Pin = XSHUT_Pin1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(XSHUT_GPIO_Port, &GPIO_InitStruct);
}
//////////////for test start //////////////////////
#if 0

uint8_t VI5300_Read_OTP(uint8_t base, uint8_t size, uint8_t* out)
{
	if(size > 32)
		return VI5300_ERROR_OTP_SIZE;

	WriteOneReg(0x0c, 0x03);
	WriteOneReg(0x0d,  size);
	WriteOneReg(0x0e,  base);
	WriteOneReg(0x0A, 0x09);
	HAL_Delay(5);
	I2C_ReadXBytes(0x0F, out, size);

	return 0;
}
#endif
//////////////for test end //////////////////////
void VI5300_Chip_PowerON(uint16_t pin)
{

	HAL_GPIO_WritePin(XSHUT_GPIO_Port, pin, GPIO_PIN_RESET);//set 0
    HAL_Delay(10);//delay 30ms
    HAL_GPIO_WritePin(XSHUT_GPIO_Port, pin, GPIO_PIN_SET);//set 1
    HAL_Delay(20);

}
uint8_t VI5300_Device_Check(void)
{
	uint8_t Chip_id = 0;
	
	ReadOneReg(0x06, &Chip_id);
	
	return Chip_id;
	
}

#ifdef CHANGE_MODEL_ADDRESS
uint8_t VI5300_Device_Adress_Modify(uint16_t pin,uint8_t addr_val)
{	
	uint8_t err_sta = 0x00;
	
	VI5300_Chip_PowerON(pin);
	if(VI5300_Device_Check()!=0xD8)
	{
		printf("Check device ID fail!\r\n");
		return 0x01; //返回错误状态
	}

	err_sta = WriteOneReg(0x01,0x88);
	if(err_sta!=0x00)
	{
		return 0x01; //返回错误状态
	} 
	err_sta = WriteOneReg(0x06,addr_val);
	if(err_sta!=0x00)
	{
		return 0x02; //返回错误状态
	} 
	gSalve = addr_val;
	VI5300_Device_Check();
	return 0x00;
}

#endif  
//////////////for test end //////////////////////
uint8_t VI5300_SetEntryStandbyMode(void)
{
		uint8_t Status = 0;

		Status = WriteOneReg(0x01, 0x0c);
    
    HAL_Delay(1);//delay 1ms
    Status = WriteOneReg(0x07, 0x0E);
    HAL_Delay(1);//delay 1ms
    Status = WriteOneReg(0x07, 0x0F);      //ADJUST TEMP DRIFT
    HAL_Delay(5);//delay 5ms

	
	 return Status;
}
void VI5300_Chip_Register_Init_Reg(void)
{
		WriteOneReg(0x00,0x00);
		WriteOneReg(0x01,0x0c);
    WriteOneReg(0x07,0x00);
    WriteOneReg(0x07,0x01);
    WriteOneReg(0x07,0x00);
		WriteOneReg(0x04, 0x21);
		WriteOneReg(0x05, 0x0E);
		WriteOneReg(0x08, 0x00);
		WriteOneReg(0x37, 0x80);
		if(chip_reg==0x00)
		{
			WriteOneReg(0x38, 0x00);
			WriteOneReg(0x3A, 0x00);
		}
		else if(chip_reg==0x30)
		{
			WriteOneReg(0x38, 0x30);
			WriteOneReg(0x3A, 0x30);

		}
		WriteOneReg(0x39, 0x00);
		WriteOneReg(0x3B, 0x80);
		WriteOneReg(0x3C, 0x80);
		WriteOneReg(0x3D, 0x80);
		WriteOneReg(0x3E, 0x00);
		WriteOneReg(0x3F, 0x00);
		WriteOneReg(0x07, 0x0E);
		WriteOneReg(0x07, 0x0F);
}

uint8_t VI5300_Wait_For_CPU_Ready(void)
{
	VI5300_Status Status = VI5300_OK;
	uint8_t stat;
	int retry = 0;

	do {
		HAL_Delay(1);//delay 1ms
		Status = ReadOneReg(0x02, &stat);
	}while((retry++ < 20)&&(stat & 0x01));
	if(retry >= 20)
	{
		//printf("CPU Busy stat = %d\n", stat);
		return 1;
	}

	return Status;
}

/********************************************************/
/*******************FOR VI5300 V3.0 20210129**********************/
VI5300_Status VI5300_Set_Digital_Clock_Dutycycle(void)
{
    VI5300_Status ret = VI5300_OK;
    
    ret = WriteOneReg(VI5300_REG_PW_CTRL, 0x0F);
		ret = WriteOneReg(VI5300_REG_PW_CTRL, 0x0E);
    VI5300_CHECK_RET(ret);
		if(chip_reg==0x00)
		{
				HAL_Delay(1);
				ret = WriteOneReg(0xE9, 0x24);
				VI5300_CHECK_RET(ret);
				ret = WriteOneReg(0xEE, 0x00);      //ADJUST TEMP DRIFT
				VI5300_CHECK_RET(ret);
				ret = WriteOneReg(0xF5, 0x00);
				VI5300_CHECK_RET(ret);
		}
		HAL_Delay(4);//默认4ms ，延时不准可以增加
    
    return ret;  
}
/*******************FOR VI5300 V3.0 20210129**********************/
/********************************************************/
 uint8_t reg_sys_cfg = 0;
VI5300_Status VI5300_WriteFirmwarePreConfig(void)
{
    VI5300_Status ret = VI5300_OK;

    VI5300_Set_Digital_Clock_Dutycycle();       // set rco config add 20210129
    
    ret = WriteOneReg(VI5300_REG_PW_CTRL, 0x08);
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_PW_CTRL, 0x0a);
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_MCU_CFG, 0x06); //0x02
    VI5300_CHECK_RET(ret);  
    ret = ReadOneReg (VI5300_REG_SYS_CFG, &reg_sys_cfg);
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_SYS_CFG, reg_sys_cfg | (0x01<<0));
    VI5300_CHECK_RET(ret);
		if(chip_reg==0x00)
		{
				ret = WriteOneReg(0x38, 0x30);
				VI5300_CHECK_RET(ret);
				ret = WriteOneReg(0x3A, 0x30);
				VI5300_CHECK_RET(ret);
		}
    ret = WriteOneReg(VI5300_REG_CMD, 0x01);
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_SIZE, 0x02);
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_SCRATCH_PAD_BASE+0x00, 0x00);
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_SCRATCH_PAD_BASE+0x01, 0x00);
    VI5300_CHECK_RET(ret);

    return VI5300_OK;
}

VI5300_Status VI5300_WriteFirmwarePostConfig(void)
{
    VI5300_Status ret = VI5300_OK;

    ret = WriteOneReg(VI5300_REG_SYS_CFG, 0x0c);//reg_sys_cfg & ~(0x01 << 0)
    VI5300_CHECK_RET(ret);
    ret = WriteOneReg(VI5300_REG_MCU_CFG, 0x06);
    VI5300_CHECK_RET(ret);
    ret = WriteOneReg(0x3B, 0xA0);
    VI5300_CHECK_RET(ret);
    ret = WriteOneReg(0x3B, 0x80);
    VI5300_CHECK_RET(ret);
		if(chip_reg==0x00)
		{
				ret = WriteOneReg(0xE9, 0x24);
				VI5300_CHECK_RET(ret);
				ret = WriteOneReg(0xEE, 0x00);         
				VI5300_CHECK_RET(ret);
				ret = WriteOneReg(0xF5, 0x00);
				VI5300_CHECK_RET(ret);
		}
    ret = WriteOneReg(VI5300_REG_MCU_CFG, 0x07);  //0x03
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_PW_CTRL, 0x02); 
    VI5300_CHECK_RET(ret);  
    ret = WriteOneReg(VI5300_REG_PW_CTRL, 0x00); 
    VI5300_CHECK_RET(ret);  

    return VI5300_OK;
}

VI5300_Status VI5300_WriteFirmware32Byte(uint8_t len, uint8_t *data)
{
    VI5300_Status ret = VI5300_OK;
    uint8_t cmd_size[2];

    if(len > 32)
        return VI5300_ERROR;

    
    cmd_size[0] = VI5300_WRITEFW_CMD;
    cmd_size[1] = len;
    
    I2C_WriteXBytes(VI5300_REG_CMD, cmd_size, 2);
    VI5300_CHECK_RET(ret);   
    
    ret = I2C_WriteXBytes(VI5300_REG_SCRATCH_PAD_BASE, data, len);
    VI5300_CHECK_RET(ret);

    return VI5300_OK; 
}

VI5300_Status VI5300_DownloadFirmware(uint8_t *buf, uint16_t size)
{
    VI5300_Status ret = VI5300_OK;
    uint8_t page = 0;

    ret = VI5300_WriteFirmwarePreConfig();
    VI5300_CHECK_RET(ret);

    while(size >= 32)
    {
        ret = VI5300_WriteFirmware32Byte(32, buf + page * 32);
        VI5300_CHECK_RET(ret);

        size -= 32;
        page++;
    }

    if(size > 0)
    {
        ret = VI5300_WriteFirmware32Byte(size, buf + page * 32);
        VI5300_CHECK_RET(ret);
    }

    ret = VI5300_WriteFirmwarePostConfig();
    VI5300_CHECK_RET(ret);

    return VI5300_OK;
}


/*****************
download firmware
******************/

void VI5300_Init_Firmware(void)
{
	VI5300_DownloadFirmware((uint8_t*)Firmware_Ranging, LoadFirmware());  
}

/***********add the function of setting integral counts and frame counts 20210105*********/
VI5300_Status VI5300_Integral_Counts_Write(uint32_t inte_counts)
{
		VI5300_Status ret = VI5300_OK;
		uint8_t buf[4]={0};
		
		//小端模式，从小到大保存
		
		buf[0] = (inte_counts)&0xFF;
		buf[1] = (inte_counts>>8)&0xFF;
		buf[2] = (inte_counts>>16)&0xFF;
		/*******************FOR VI5300 V3.0**********************/
    /********************************************************/
    VI5300_Set_Digital_Clock_Dutycycle();       // set rco config add 20210129
    /*******************FOR VI5300 V3.0**********************/
    /********************************************************/
		ret = WriteOneReg(0x0C, 0x01);
		VI5300_CHECK_RET(ret);
		ret = WriteOneReg(0x0D, 0x03);
		VI5300_CHECK_RET(ret);
		ret = WriteOneReg(0x0E, 0x01);
		VI5300_CHECK_RET(ret);
		//小端模式，从小到大保存
		//ret = WriteOneReg(0x0F, *((uint8_t *)(&inte_counts)));
		ret = WriteOneReg(0x0F, buf[0]);
		VI5300_CHECK_RET(ret);
		//ret = WriteOneReg(0x0F, *((uint8_t *)(&inte_counts)+ 1));
		ret = WriteOneReg(0x10, buf[1]);
		VI5300_CHECK_RET(ret);
		ret = WriteOneReg(0x11, buf[2]);
		//ret = WriteOneReg(0x11, *((uint8_t *)(&inte_counts) + 2));
		VI5300_CHECK_RET(ret);
		ret = WriteOneReg(0x0A, 0x09);
		VI5300_CHECK_RET(ret);

	return ret;
}
	

VI5300_Status VI5300_Delay_Count_Write(uint16_t delay_count)
{
	VI5300_Status ret = VI5300_OK;
	uint8_t buf[2]={0};
	
		//大端模式
	buf[0] = (delay_count>>8)&0xFF;
	buf[1] = (delay_count)&0xFF;
	 
	VI5300_Set_Digital_Clock_Dutycycle();       // set rco config add 20210129
  
	ret = WriteOneReg(0x0C, 0x01);
	VI5300_CHECK_RET(ret);
	ret = WriteOneReg(0x0D, 0x02);
	VI5300_CHECK_RET(ret);
	ret = WriteOneReg(0x0E, 0x04);
	VI5300_CHECK_RET(ret);
	//大端模式
	
	ret = WriteOneReg(0x0F, buf[0]);
	VI5300_CHECK_RET(ret);
	ret = WriteOneReg(0x10, buf[1]);
	VI5300_CHECK_RET(ret);
	ret = WriteOneReg(0x0A, 0x09);
	VI5300_CHECK_RET(ret);
	
	return ret;
}

VI5300_Status VI5300_Set_Integralcounts_Frame(uint8_t fps, uint32_t intecoutns)
{
	VI5300_Status ret = VI5300_OK;
	uint32_t inte_time;
	uint32_t fps_time;
	uint32_t delay_time;
	uint16_t delay_counts;

	inte_time = intecoutns *1463/10;
	fps_time = 1000000000/fps;
	delay_time = fps_time - inte_time -1600000;
	delay_counts = (uint16_t)(delay_time/40900);

	ret = VI5300_Integral_Counts_Write(intecoutns);
	VI5300_CHECK_RET(ret);
	ret = VI5300_Delay_Count_Write(delay_counts);
	VI5300_CHECK_RET(ret);

	return ret;
}
/***********add the function of setting integral counts and frame counts 20210105*********/

/***********add the function of starting xtalk calibration 20210105*** ******/
//在测距之前执行xtalk标定，标定结束之后将结果存储，在每次测距之前将调用VI5300_Config_XTalk_Parameter（）
//函数将xtalk参数xtalk_cal.postion配置进芯片，芯片不下电，无需重新配置。
VI5300_XTALK_Calib_Data  xtalk_cal;
VI5300_Status VI5300_Start_XTalk_Calibration(void)
{
	VI5300_Status ret = VI5300_OK;
	uint8_t status;
	uint8_t cg_buf[3];
	
	VI5300_Wait_For_CPU_Ready();
	/*******************FOR VI5300 V3.0**********************/
    /********************************************************/
    VI5300_Set_Digital_Clock_Dutycycle();       // set rco config add 20210129
    /*******************FOR VI5300 V3.0**********************/
    /********************************************************/
	WriteOneReg(0x0A, 0x0D);
	HAL_Delay(100);
	while(1) {
		ReadOneReg(VI5300_RET_INT_STATUS, &status);
		if((status & 0x01) == 0x01)
		{
			//缓存都是小端模式
			I2C_ReadXBytes(VI5300_REG_SCRATCH_PAD_BASE, cg_buf, 3);
			xtalk_cal.xtalk_cal = (uint8_t)cg_buf[0];
			xtalk_cal.xtalk_peak = (uint16_t)((((uint16_t)cg_buf[2])<<8) |(( (uint16_t)cg_buf[1])));
			
			break;
		}
		HAL_Delay(50);
	}

	return ret;
}
void VI5300_Get_XTalk_Parameter(uint8_t  *position)
{
	
	*position = xtalk_cal.xtalk_cal;
	
}
uint8_t VI5300_Stop_Continuous_Measure(void)
{
    VI5300_Status ret = VI5300_OK;
    
    ret = WriteCommand(0x1F);
    VI5300_CHECK_RET(ret);

    return ret;
}
VI5300_Status VI5300_Get_Interrupt_State(uint8_t *status)
{
    return ReadOneReg(VI5300_RET_INT_STATUS, status);
}

VI5300_Status VI5300_Start_Continuous_Measure(void)
{    
    VI5300_Status ret = VI5300_OK;
		uint8_t stat = 0;
	
		ret= VI5300_Get_Interrupt_State(&stat);
		
    /*******************FOR VI5300 V3.0**********************/
    /********************************************************/
		VI5300_Wait_For_CPU_Ready();
		#ifdef VI5300_XTALK_CALIBRATION
	  VI5300_Config_XTalk_Parameter();       //这一步依情况而定，如果sensor有coverglass，则需coverglass标定，如果没有则不需要打开 20210105
		#endif

    VI5300_Set_Digital_Clock_Dutycycle();
    ret = WriteCommand(0x0F);
    VI5300_CHECK_RET(ret);

    return VI5300_OK;
}
VI5300_Status VI5300_Start_Single_Measure(void)
{    
    VI5300_Status ret = VI5300_OK;
		uint8_t stat = 0;
		ret= VI5300_Get_Interrupt_State(&stat);
	
    /*******************FOR VI5300 V3.0**********************/
    /********************************************************/
		VI5300_Wait_For_CPU_Ready();
		#ifdef VI5300_XTALK_CALIBRATION
		VI5300_Config_XTalk_Parameter();       //这一步依情况而定，如果sensor有coverglass，则需coverglass标定，如果没有则不需要打开 20210105
		#endif

    VI5300_Set_Digital_Clock_Dutycycle();
    ret = WriteCommand(0x0E);
    VI5300_CHECK_RET(ret);

    return VI5300_OK;
}
/***********add the function of offset calibration 20210119**** *****/
VI5300_Status VI5300_Config_XTalk_Parameter(void)
{
	VI5300_Status ret = VI5300_OK;
	
	VI5300_Wait_For_CPU_Ready();
	/*******************FOR VI5300 V3.0**********************/
    /********************************************************/
  VI5300_Set_Digital_Clock_Dutycycle();       // set rco config add 20210129
    /*******************FOR VI5300 V3.0**********************/
    /********************************************************/
	WriteOneReg(0x0C, 0x01);
	WriteOneReg(0x0D, 0x01);
	WriteOneReg(0x0E, 0x00);
	WriteOneReg(0x0F, xtalk_cal.xtalk_cal);
	WriteOneReg(0x0A, 0x09);

	return ret;
}
/***********add the function of starting xtalk calibration 20210105**** *****/

/***********add the function of offset calibration 20210119**** *****/
//在测距之前执行offset标定，表姐结束之后将标定结果存储，下一次将标定结果取出来，参与tof结果的计算
VI5300_OFFSET_Calib_Data  offset_cal;
VI5300_Status VI5300_Start_Offset_Calibration(void)
{
	VI5300_Status ret = VI5300_OK;
	uint8_t buf[32];
	int16_t milimeter;
	uint32_t peak1,peak2,peak_tmp;
	uint32_t integral_times;
	int32_t bias;
	uint32_t peak_t;
	int16_t total = 0;
	int16_t offset = 0;
	int cnt = 0;
	uint8_t stat = 0;
	
	ret = VI5300_Start_Continuous_Measure();
	VI5300_CHECK_RET(ret);
	while(1)
	{
		HAL_Delay(35);
		VI5300_Get_Interrupt_State(&stat);
		if((stat & 0x01) == 0x01)
		{
			I2C_ReadXBytes(VI5300_REG_SCRATCH_PAD_BASE, buf, 32);
			milimeter = (int16_t)((((int16_t)buf[13])<<8) |(( (int16_t)buf[12])));
//		peak1 = (uint32_t)((((uint32_t)buf[31])<<24) |((uint32_t)buf[30]<<16)|((uint32_t)buf[29]<<8)|((uint32_t)buf[28]));
//		noise = (uint16_t)((((uint16_t)buf[27])<<8) |(((uint16_t) buf[26])));
			peak2 = (uint32_t)((((uint32_t)buf[11])<<24) |( ((uint32_t)buf[10])<<16)|(((uint32_t)buf[9])<<8)|( (uint32_t)buf[8]));

			integral_times = (uint32_t)((((uint32_t)buf[25])<<24) |( ((uint32_t)buf[24])<<16)|( ((uint32_t)buf[23])<<8)|( (uint32_t)buf[22]));
			integral_times = integral_times & 0x00ffffff;
			if(peak2 > 65536) {
					peak_tmp = (peak2 * 256) / integral_times * 256;
			} else {
					peak_tmp = peak2 * 65536 / integral_times;
			}

			peak_tmp = peak_tmp >> 12;
			bias = (int32_t)(PILEUP_A / (PILEUP_B - peak_tmp * PILEUP_D) - PILEUP_C) / PILEUP_D;
			if(bias < 0)
			{
				bias = 0;
			}
			milimeter = milimeter + (int16_t)bias;
				
				total += milimeter;
				++cnt;
			} 
			else
				continue;
			if(cnt >= 30)
				break;
	}
	VI5300_Stop_Continuous_Measure();
	VI5300_CHECK_RET(ret);
	offset = total / 30;
	offset_cal.offset_cal = offset - 100;   //此处100是指在10cm处做offset标定，在多远距离做标定减多少，单位mm
	
	return VI5300_OK;
}

	
VI5300_Status VI5300_GetRawRangingData(VI5300_Dist_TypeDef *result)
{
    VI5300_Status ret = VI5300_OK;
	
    uint8_t buf[32];
    int16_t milimeter;
    uint16_t noise;
    uint32_t peak1;
    uint32_t peak2;
    uint32_t integral_times;
    uint32_t peak_tmp;
    uint16_t noise_tmp;
	uint32_t lower;
	uint32_t upper;
    int32_t bias;
    uint32_t confidence;

    ret = I2C_ReadXBytes(VI5300_REG_SCRATCH_PAD_BASE, buf, 32);
    VI5300_CHECK_RET(ret);

	/***********add the function of pileup and confidence 20210105  小端模式*********/
		milimeter = (short)((((short)buf[13])<<8) |(( (short)buf[12])));
		peak1 = (uint32_t)((((uint32_t)buf[31])<<24) |((uint32_t)buf[30]<<16)|((uint32_t)buf[29]<<8)|((uint32_t)buf[28]));
		noise = (uint16_t)((((uint16_t)buf[27])<<8) |(((uint16_t) buf[26])));
		peak2 = (uint32_t)((((uint32_t)buf[11])<<24) |( ((uint32_t)buf[10])<<16)|(((uint32_t)buf[9])<<8)|( (uint32_t)buf[8]));

	integral_times = (uint32_t)((((uint32_t)buf[25])<<24) |( ((uint32_t)buf[24])<<16)|( ((uint32_t)buf[23])<<8)|( (uint32_t)buf[22]));
    integral_times = integral_times & 0x00ffffff;
    if(peak2 > 65536) {
        peak_tmp = (peak2 * 256) / integral_times * 256;
    } else {
        peak_tmp = peak2 * 65536 / integral_times;
    }

    peak_tmp = peak_tmp >> 12;
    bias = (int32_t)(PILEUP_A / (PILEUP_B - peak_tmp * PILEUP_D) - PILEUP_C) / PILEUP_D;
    if(bias < 0)
	{
		bias = 0;
	}
	milimeter = milimeter + (int16_t)bias;
	
#ifdef VI5300_OFFSET_CALIBRATION
	milimeter = milimeter - offset_cal.offset_cal;//减去offset
#endif
	
	noise_tmp = noise  / 8;
	if(noise_tmp < 45) {
		lower = 228 * noise / 8 + 1260;
		upper = 269 * noise / 8 + 2284;
	} else if(noise_tmp < 94) {
		lower = 252 * noise / 8 + 172;
		upper = 300 * noise / 8 + 901;
	} else if(noise_tmp <154) {
		lower = 190 * noise / 8 + 6066;
		upper = 227 * noise / 8 + 7703;
	} else if(noise_tmp <246) {
		lower = 209 * noise / 8 + 3043;
		upper = 237 * noise / 8 + 6121;
	} else if(noise_tmp <721){
		lower = 197 * noise / 8 + 6006;
		upper = 216 * noise / 8 + 11300;
	} else if(noise_tmp <1320){
		lower = 201 * noise / 8 + 3177;
		upper = 226 * noise / 8 + 4547;
	} else if(noise_tmp <2797){
		lower = 191 * noise / 8 + 16874;
		upper = 217 * noise / 8 + 16340;
	} else if(noise_tmp <3581){
		lower = 188 * noise / 8 + 23925;
		upper = 218 * noise / 8 + 14012;
	} else {
		lower = 180 * noise / 8 + 53524;
		upper = 204 * noise / 8 + 62305;
	}
	if(peak1 < lower) {
		confidence = 0;
	} else if(peak1 > upper) {
		confidence = 100;
	} else {
		confidence = 100 * (peak1 - lower) / (upper - lower);
	}

	result->millimeter = milimeter;
    result->peak       = peak1;
    result->noise      = noise;
	result->confidence = confidence;

    return VI5300_OK;
}


VI5300_Status VI5300_Get_Measure_Data(VI5300_Dist_TypeDef *result)
{
    VI5300_Status ret = VI5300_OK;
  
	uint8_t int_status = 0;
    ret = VI5300_Get_Interrupt_State(&int_status);
    VI5300_CHECK_RET(ret);

    if((int_status & 0x01) == 0x00)
    {
        return VI5300_RANGING;
    }
    ret = VI5300_GetRawRangingData(result);
    VI5300_CHECK_RET(ret);

    return VI5300_OK;
}
VI5300_Status VI5300_Interrupt_Enable(void)
{
	VI5300_Status Status = VI5300_OK;
	int loop = 0;
	uint8_t enable = 0;

	do
	{
		Status |= ReadOneReg(0x04,&enable);
		enable |=  0x01;
		Status |= WriteOneReg(0x04, enable);
		Status |= ReadOneReg(0x04, &enable);
		loop++;
	} while((loop < 5)&& (!(enable & 0x01)));
	if(loop >= 5)
	{
	   rt_kprintf("Enable interrupt Failed!");
        
       Status = VI5300_ERROR;
	}

    return Status;
}

void VI5300_init(void)
{
	uint8_t stat = 0;
	uint8_t retry=0;;
	
	for(retry = 0; retry < 5; retry++)
	{
		ReadOneReg(0x38, &chip_reg38);
		if(chip_reg38 == 0x30)
		{
			ReadOneReg(0x3a, &chip_reg3a);
			if(chip_reg3a == 0x30)
			{
				chip_reg=0x30;                                                                              
				break;
			} 
		} 
		else if(chip_reg38 == 0x00)
		{
			ReadOneReg(0x3a, &chip_reg3a);
			if(chip_reg3a == 0x00)
			{
				chip_reg=0x00;                                                                                  
				break;
			} 
		}
	}
	VI5300_Chip_Register_Init_Reg();
	VI5300_Wait_For_CPU_Ready();
	ReadOneReg(0x08, &stat);
	HAL_Delay(1);
	do 
	{
		WriteOneReg(0x08, 0x00);
		HAL_Delay(1);
		ReadOneReg(0x08, &stat);
	}while(stat!=0&&retry++<5);
	retry=0;
	do
	{
		VI5300_Init_Firmware();
		HAL_Delay(10);//delay 10ms
		ReadOneReg(0x08, &stat);
	}while((stat!=0x55)&&(stat!=0x66)&&(retry++<5));
	if(retry>=5)
	{
		rt_kprintf("Download firmware fail!\r\n");
		return ;
	}
	VI5300_Interrupt_Enable();
	VI5300_Set_Integralcounts_Frame(30, 131072);
}

