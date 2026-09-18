#include "bsp_dwt.h"

//FreeRTOS下SysTick将被RTOS接管和配置,这里使用DWT定时器满足延迟函数
//DWT计数器频率为系统时钟频率,计数器共32位,顺序计数.
void DWT_Init(void)
{
	//1.使用DWT前必须使能DGB的系统跟踪（《Cortex-M3权威指南》）
	DEM_CR |= 1<<24;

	//2.计数器清0
	DWT_CYCCNT = (uint32_t)0u;

	//3.开启计时
	DWT_CTRL |= 1<<0;
}

//返回32位计数器CYCCNT的值,计数频率为系统时钟频率
static uint32_t DWT_GetTick(void)
{
	return (uint32_t)DWT_CYCCNT;
}

void delay_us(uint32_t us)
{
	//超出能满足的最大延迟 25565281
	if( us > DWT_CNT_MAX/(SystemCoreClock/1000000) ) us = DWT_CNT_MAX/(SystemCoreClock/1000000);
	
	us = us*(SystemCoreClock/1000000); //单位转换
	
	//用于保存已走过的时间
	uint32_t runningtime = 0;
	
	//获得当前时刻的计数值
	uint32_t InserTick = DWT_GetTick();
	
	//用于刷新实时时间
	uint32_t tick = 0;
	
	uint8_t countflag = 0;
	//等待延迟
	while(1)
	{
		tick = DWT_GetTick();//刷新当前时刻计数值
		
		if( tick < InserTick ) countflag = 1;//出现溢出轮询,则切换走时的计算方式
		
		if( countflag ) runningtime = tick + DWT_CNT_MAX - InserTick;
		else runningtime = tick - InserTick;
		
		if( runningtime>=us ) break;
	}
}

void delay_ms(uint32_t ms)
{
	//超出能满足的最大延迟
	if( ms > DWT_CNT_MAX/(SystemCoreClock/1000) ) ms = DWT_CNT_MAX/(SystemCoreClock/1000);
	delay_us(ms*1000);
}

