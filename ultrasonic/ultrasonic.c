#include "ultrasonic.h"

Ultrasonic ultrasonic_left,ultrasonic_right;

//主频168M延时函数
void delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}

void Ultrasonic_Init()
{
    ultrasonic_left.distance = 0;
    ultrasonic_left.rising_time = 0;
    ultrasonic_left.falling_time = 0;
    ultrasonic_left.capture_state = 0;
    ultrasonic_left.time = 0;
    ultrasonic_left.loop_num = 0;
    ultrasonic_left.finish = 0;

    ultrasonic_right.distance = 0;
    ultrasonic_right.rising_time = 0;
    ultrasonic_right.falling_time = 0;
    ultrasonic_right.capture_state = 0;
    ultrasonic_right.time = 0;
    ultrasonic_right.loop_num = 0;
    ultrasonic_right.finish = 0;

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);	 //开启捕获中断
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);	 //开启捕获中断
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);	 //开启溢出中断
}


//20us的高电平
void Ultrasonic_Start()
{
    Trig1(0);
    Trig2(0);
    delay_us(20);
    Trig1(1);
    Trig2(1);

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
}

//重写输入捕获中断
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim3.Instance)
	{
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            if(ultrasonic_left.capture_state == 0) 		//如果当前状态为捕获高电平																							
            {
                ultrasonic_left.rising_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);	//上升沿的时间								
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3,TIM_INPUTCHANNELPOLARITY_FALLING);	//开始捕获低电平
                HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);	 																								//重新开启输入捕获中断
                
                ultrasonic_left.capture_state = 1;		//将状态改为捕获低电平																												
            }
            else if(ultrasonic_left.capture_state == 1)		//如果当前状态为捕获低电平																									
            {
                ultrasonic_left.falling_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);				//获得下降沿的时间
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);			//恢复为捕获高电平
                HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_3);																									//停止输入捕获，等触发信号开始再开启
            
                ultrasonic_left.time = ultrasonic_left.falling_time + ultrasonic_left.loop_num * RELOADVALUE - ultrasonic_left.rising_time;	
                ultrasonic_left.distance = (float)ultrasonic_left.time/57.5;	//转换为cm为单位
                //清空状态
                ultrasonic_left.loop_num = 0;
                ultrasonic_left.capture_state = 0;

                ultrasonic_left.finish = 1;
            }
        }

        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            if(ultrasonic_right.capture_state == 0) 		//如果当前状态为捕获高电平																							
            {
                ultrasonic_right.rising_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);	//上升沿的时间								
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4,TIM_INPUTCHANNELPOLARITY_FALLING);	//开始捕获低电平
                HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);	 																								//重新开启输入捕获中断
                
                ultrasonic_right.capture_state = 1;		//将状态改为捕获低电平																												
            }
            else if(ultrasonic_right.capture_state == 1)		//如果当前状态为捕获低电平																									
            {
                ultrasonic_right.falling_time = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);				//获得下降沿的时间
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);			//恢复为捕获高电平
                HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_4);																									//停止输入捕获，等触发信号开始再开启
            
                ultrasonic_right.time = ultrasonic_right.falling_time + ultrasonic_right.loop_num * RELOADVALUE - ultrasonic_right.rising_time;	
                ultrasonic_right.distance = (float)ultrasonic_right.time/57.5;	//转换为cm为单位
                //清空状态
                ultrasonic_right.loop_num = 0;
                ultrasonic_right.capture_state = 0;

                ultrasonic_right.finish = 1;
            }
        }
	}
}

//在更新中断中，我们要处理溢出中断
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if(htim->Instance == htim3.Instance)	//溢出中断
//     {
//         if(ultrasonic_left.capture_state == 1)	//此时是在捕获低电平
//         {
//             ultrasonic_left.loop_num++;	
//         }
//         if(ultrasonic_right.capture_state == 1)	//此时是在捕获低电平
//         {
//             ultrasonic_right.loop_num++;	
//         }
//     }
// }

void Ultrasonic_TIM_PeriodElapsedCallback()
{
    if(ultrasonic_left.capture_state == 1)	//此时是在捕获低电平
    {
        ultrasonic_left.loop_num++;	
    }
    if(ultrasonic_right.capture_state == 1)	//此时是在捕获低电平
    {
        ultrasonic_right.loop_num++;	
    }
}

void Ultrasonic_clearFlag()
{
    ultrasonic_left.finish = 0;
    ultrasonic_right.finish = 0;
}

bool Ultrasonic_canPublish()
{
    return (ultrasonic_left.finish || ultrasonic_right.finish);
}

float Ultrasonic_getLeftDistance()
{
    return ultrasonic_left.distance;
}

float Ultrasonic_getRightDistance()
{
    return ultrasonic_right.distance;
}
