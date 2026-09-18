#include "bsp_buzzer.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

static uint8_t TaskTipsCount = 0;   //蜂鸣次数
static uint16_t TaskTipsTimeInt = 0;//蜂鸣时间间隔
volatile static uint8_t taskRunning = 0;

static void buzzer_init(void)
{
	
}

static void buzzer_on(void)
{
	HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin,GPIO_PIN_SET);
}

static void buzzer_off(void)
{
	HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin,GPIO_PIN_RESET);
}

static void buzzer_toggle(void)
{
	HAL_GPIO_TogglePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin);
}

static void buzzer_start_task(void* param)
{
	for(uint8_t i=0;i<TaskTipsCount;i++)
	{
		buzzer_on();
		vTaskDelay( pdMS_TO_TICKS(TaskTipsTimeInt) );
		buzzer_off();
		vTaskDelay( pdMS_TO_TICKS(TaskTipsTimeInt) );
	}
	
	taskRunning=0;
	vTaskDelete(NULL); //设置完成,删除任务
}

static void buzzer_SetTask(uint8_t tipscnt,uint16_t time_int )
{
	if( taskRunning==1 ) return;
	TaskTipsCount=tipscnt;
	TaskTipsTimeInt = time_int;
	taskRunning=1;
	xTaskCreate(buzzer_start_task,"buzzerTask",64,NULL,osPriorityNormal,NULL);
}

BuzzerInterface_t UserBuzzer = {
	.init = buzzer_init,
	.on = buzzer_on,
	.off = buzzer_off,
	.toggle = buzzer_toggle,
	.AddTask = buzzer_SetTask
};

