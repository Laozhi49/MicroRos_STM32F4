#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include <stdbool.h>

#define CPU_FREQUENCY_MHZ    168		// F407主频，用来写20us延时函数

#define Trig1(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, (GPIO_PinState)(state))  //触发引脚
#define Trig2(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, (GPIO_PinState)(state))  //触发引脚

#define RELOADVALUE 65535	//重装载值

typedef struct
{
	float distance;						//计算出来的距离
	uint8_t loop_num;					//溢出次数
	uint32_t rising_time;				//捕获上升沿的时间
	uint32_t falling_time;				//捕获下降沿的时间
	uint8_t capture_state;				//当前捕获的状态
	uint32_t time;						//计算出来的时间
	uint8_t finish;
}Ultrasonic;

//声明一下回调函数
extern void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
// extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Ultrasonic_TIM_PeriodElapsedCallback();

void Ultrasonic_Init();
void Ultrasonic_Start();
void Ultrasonic_clearFlag();
bool Ultrasonic_canPublish();
float Ultrasonic_getLeftDistance();
float Ultrasonic_getRightDistance();

#endif


