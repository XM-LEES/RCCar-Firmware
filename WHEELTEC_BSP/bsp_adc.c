#include "bsp_adc.h"
#include "adc.h"

#define userconfig_ADCDMA_BUF_LEN 80 //设置dma搬运某个通道的个数

static uint16_t g_Adc1Buf[userconfig_ADCDMA_BUF_LEN][2] = { 0 };

void ADC_Userconfig_Init(void)
{
	//启动ADC采集并设置DMA传输
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)g_Adc1Buf,sizeof(g_Adc1Buf)/sizeof(g_Adc1Buf[0][0]));
}

//获取DMA搬运后的ADC值
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
	uint32_t tmp = 0;
	
	//索引判断防止数组溢出
	if( channel > 1 ) return 0;
	
	for(uint8_t i=0;i<userconfig_ADCDMA_BUF_LEN;i++)
	{
		tmp += g_Adc1Buf[i][channel];
	}
	
	return tmp/userconfig_ADCDMA_BUF_LEN;
}

